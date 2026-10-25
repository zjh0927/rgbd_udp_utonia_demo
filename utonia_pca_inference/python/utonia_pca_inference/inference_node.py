import copy
import os
import threading
import time
from typing import Optional

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from .cloud_io import CloudFormatError, CloudFrame, decode_xyzrgb, make_xyzrgb
from .config import InferenceConfig
from .fixed_pca import FixedPca, PcaError, PcaIdentity
from .latest_slot import LatestSlot, SlotClosed
from .metrics import Metrics
from .utonia_backend import BackendError, CudaOutOfMemory, UtoniaBackend


class InferenceNode(Node):
    def __init__(self, direct_input: bool = False):
        super().__init__("utonia_pca_inference")
        self._direct_input = direct_input
        self.config = InferenceConfig.from_node(self)
        self._validate_paths()
        self._slot: LatestSlot[CloudFrame] = LatestSlot()
        self._metrics = Metrics()
        self._backend: Optional[UtoniaBackend] = None
        self._pca: Optional[FixedPca] = None
        self._worker: Optional[threading.Thread] = None
        self._started = False
        self._closed = False
        self._paused_oom = False
        self._consecutive_oom = 0
        self._dynamic_max_points = self.config.max_model_points
        self._last_log = {}
        self._last_metric_counts = {}
        self._last_metrics_time = time.monotonic()
        self._publisher = None
        self._subscription = None
        self._metrics_timer = None
        self._publisher = self.create_publisher(
            PointCloud2, self.config.output_topic, qos_profile_sensor_data
        )
        if not self._direct_input:
            self._subscription = self.create_subscription(
                PointCloud2,
                self.config.input_topic,
                self._on_cloud,
                qos_profile_sensor_data,
            )
        self._metrics_timer = self.create_timer(
            self.config.metrics_interval_ms / 1000.0, self._report_metrics
        )

    def _validate_paths(self) -> None:
        source = os.path.abspath(os.path.expanduser(self.config.utonia_source_path))
        checkpoint = os.path.abspath(os.path.expanduser(self.config.checkpoint_path))
        if not os.path.isdir(os.path.join(source, "utonia")):
            raise ValueError(f"Utonia source tree not found: {source}")
        if not os.path.isfile(checkpoint):
            raise ValueError(f"Utonia checkpoint not found: {checkpoint}")

    def start(self) -> None:
        if self._started:
            raise RuntimeError("inference node already started")
        self.get_logger().info("Loading Utonia Tiny and running CUDA warmup...")
        self._backend = UtoniaBackend(self.config)
        warmup = self._backend.warmup()
        feature_dim = int(warmup.features.shape[1])
        identity = PcaIdentity(
            model_sha256=self._backend.model_sha256,
            feature_dim=feature_dim,
            utonia_scale=self.config.utonia_scale,
            max_model_points=self.config.max_model_points,
        )
        self._pca = FixedPca(
            profile_path=self.config.pca_profile_path,
            calibration_frames=self.config.pca_calibration_frames,
            max_samples=self.config.pca_max_samples,
            identity=identity,
            random_seed=self.config.random_seed,
        )
        if self._pca.loaded_existing:
            self.get_logger().info(
                f"Loaded fixed PCA profile: {self.config.pca_profile_path}"
            )
        else:
            if self._pca.load_error:
                self.get_logger().warning(
                    "Ignoring incompatible PCA profile and recalibrating: "
                    + self._pca.load_error
                )
            self.get_logger().info(
                f"PCA calibration will collect {self.config.pca_calibration_frames} "
                "successful inference frames before publishing"
            )
        del warmup
        self._worker = threading.Thread(
            target=self._inference_loop, name="utonia-gpu-inference", daemon=True
        )
        self._worker.start()
        self._started = True
        self.get_logger().info(
            f"Utonia inference ready: "
            f"{'direct NumPy input' if self._direct_input else self.config.input_topic} -> "
            f"{self.config.output_topic}, max_points={self.config.max_model_points}, "
            f"feature_dim={feature_dim}, model_sha256={identity.model_sha256[:12]}..."
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        started = time.perf_counter()
        try:
            frame = decode_xyzrgb(message)
        except (CloudFormatError, ValueError) as exc:
            self._metrics.increment("input_errors")
            self._log_throttled("input", f"Rejected input PointCloud2: {exc}")
            return
        self._enqueue_frame(frame)
        self._metrics.observe_ms("decode", (time.perf_counter() - started) * 1000.0)

    def submit_points(self, points, timestamp_ns: int, frame_id: str) -> None:
        """Pass projected NumPy arrays directly from the receiver worker."""
        stamp = Time(
            sec=int(timestamp_ns // 1_000_000_000),
            nanosec=int(timestamp_ns % 1_000_000_000),
        )
        frame = CloudFrame(
            xyz=points.xyz,
            rgb=points.rgb,
            header=Header(stamp=stamp, frame_id=frame_id),
        )
        self._enqueue_frame(frame)

    def _enqueue_frame(self, frame: CloudFrame) -> None:
        try:
            overwritten = self._slot.put(frame)
        except SlotClosed:
            return
        self._metrics.increment("input_frames")
        self._metrics.gauge("input_points", frame.count)
        if overwritten:
            self._metrics.increment("slot_overwrites")

    def _inference_loop(self) -> None:
        assert self._backend is not None
        assert self._pca is not None
        while True:
            taken = self._slot.wait_take()
            if taken is None:
                return
            if self._paused_oom:
                self._metrics.increment("paused_drops")
                continue
            frame = taken.value
            try:
                inference_started = time.perf_counter()
                feature_frame = self._backend.infer(
                    frame, max_points=self._dynamic_max_points
                )
                self._metrics.observe_ms(
                    "utonia", (time.perf_counter() - inference_started) * 1000.0
                )
                self._metrics.increment("inference_frames")
                self._metrics.gauge("model_points", feature_frame.count)

                pca_started = time.perf_counter()
                colors = self._pca.observe(feature_frame.features)
                self._metrics.observe_ms(
                    "pca", (time.perf_counter() - pca_started) * 1000.0
                )
                self._metrics.gauge("pca_calibration_frames", self._pca.observed_frames)
                self._consecutive_oom = 0
                if colors is None:
                    self._metrics.increment("calibration_frames")
                    continue

                header = copy.deepcopy(feature_frame.header)
                if self.config.output_frame_id:
                    header.frame_id = self.config.output_frame_id
                output_frame = CloudFrame(
                    xyz=feature_frame.xyz,
                    rgb=colors,
                    header=header,
                )
                publish_started = time.perf_counter()
                message = make_xyzrgb(output_frame, colors)
                self._publisher.publish(message)
                self._metrics.observe_ms(
                    "publish", (time.perf_counter() - publish_started) * 1000.0
                )
                self._metrics.increment("published_frames")
                self._metrics.gauge("output_points", feature_frame.count)
            except CudaOutOfMemory as exc:
                self._handle_oom(exc)
            except (BackendError, PcaError, ValueError) as exc:
                self._consecutive_oom = 0
                self._metrics.increment("inference_errors")
                self._log_throttled("inference", f"Dropped inference frame: {exc}")
            except Exception as exc:
                if self._is_cuda_oom(exc):
                    self._handle_oom(exc)
                    continue
                self._consecutive_oom = 0
                self._metrics.increment("unexpected_errors")
                self._log_throttled(
                    "unexpected",
                    f"Unexpected inference error {type(exc).__name__}: {exc}",
                    level="error",
                )

    def _is_cuda_oom(self, error: Exception) -> bool:
        """Recognize CUDA OOMs raised outside the model backend, e.g. PCA."""
        if self._backend is None:
            return False
        oom_type = getattr(self._backend.torch.cuda, "OutOfMemoryError", ())
        if oom_type and isinstance(error, oom_type):
            return True
        message = str(error).lower()
        return "cuda" in message and "out of memory" in message

    def _handle_oom(self, error: Exception) -> None:
        assert self._backend is not None
        self._metrics.increment("cuda_oom")
        self._consecutive_oom += 1
        self._backend.empty_cache()
        old_limit = self._dynamic_max_points
        self._dynamic_max_points = max(
            self.config.oom_min_points, self._dynamic_max_points // 2
        )
        if self._consecutive_oom >= self.config.oom_max_consecutive:
            self._paused_oom = True
            self.get_logger().error(
                f"Paused Utonia after {self._consecutive_oom} consecutive CUDA OOMs; "
                "reduce max_model_points and restart the node. Last error: "
                f"{error}"
            )
            return
        self._log_throttled(
            "oom",
            f"CUDA OOM; dropped frame and reduced point limit "
            f"{old_limit}->{self._dynamic_max_points}: {error}",
            level="error",
            period_seconds=1.0,
        )

    def _log_throttled(
        self,
        key: str,
        message: str,
        level: str = "warning",
        period_seconds: float = 5.0,
    ) -> None:
        now = time.monotonic()
        if now - self._last_log.get(key, 0.0) < period_seconds:
            return
        self._last_log[key] = now
        getattr(self.get_logger(), level)(message)

    def _report_metrics(self) -> None:
        now = time.monotonic()
        elapsed = max(now - self._last_metrics_time, 1e-6)
        current = self._metrics.snapshot()

        def rate(name: str) -> float:
            value = int(current.get(name, 0))
            previous = int(self._last_metric_counts.get(name, 0))
            return (value - previous) / elapsed

        self.get_logger().info(
            "fps input={:.1f} infer={:.1f} publish={:.1f} | points input={} model={} "
            "output={} limit={} | overwrite={} errors input={} infer={} oom={} paused={} | "
            "ms decode={:.2f}/{:.2f} utonia={:.2f}/{:.2f} pca={:.2f}/{:.2f} "
            "publish={:.2f}/{:.2f}".format(
                rate("input_frames"),
                rate("inference_frames"),
                rate("published_frames"),
                current.get("input_points", 0),
                current.get("model_points", 0),
                current.get("output_points", 0),
                self._dynamic_max_points,
                current.get("slot_overwrites", 0),
                current.get("input_errors", 0),
                current.get("inference_errors", 0)
                + current.get("unexpected_errors", 0),
                current.get("cuda_oom", 0),
                int(self._paused_oom),
                current.get("decode_avg_ms", 0.0),
                current.get("decode_p95_ms", 0.0),
                current.get("utonia_avg_ms", 0.0),
                current.get("utonia_p95_ms", 0.0),
                current.get("pca_avg_ms", 0.0),
                current.get("pca_p95_ms", 0.0),
                current.get("publish_avg_ms", 0.0),
                current.get("publish_p95_ms", 0.0),
            )
        )
        self._last_metric_counts = current
        self._last_metrics_time = now

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._slot.close()
        if self._worker is not None:
            self._worker.join(timeout=10.0)
            if self._worker.is_alive():
                self.get_logger().error(
                    "GPU inference worker did not stop within 10 seconds"
                )
