import threading
import time
from typing import Callable, List, Optional

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from .calibration import CalibrationError, CalibrationStore
from .config import ReceiverConfig
from .h265_decoder import H265DecodeError, H265Decoder
from .latest_slot import LatestSlot, SlotClosed
from .metrics import Metrics
from .models import ColoredPoints, CompleteFrame, DecodedBundle, StreamType
from .pairing import PairingBuffer
from .pointcloud2 import make_pointcloud2
from .projection import ProjectionError, project_color_depth
from .udp_ingress import UdpIngress
from .zdepth_decoder import ZdepthDecodeError, ZdepthDecoder


class ReceiverNode(Node):
    def __init__(
        self,
        point_callback: Optional[Callable[[ColoredPoints, int, str], None]] = None,
    ):
        super().__init__("rgbd_udp_receiver")
        self._point_callback = point_callback
        self.config = ReceiverConfig.from_node(self)
        self._metrics = Metrics()
        self._calibrations = CalibrationStore(
            self.config.max_calibration_generations,
            self.config.expected_width,
            self.config.expected_height,
        )
        self._pairing = PairingBuffer(
            self._calibrations,
            timeout_ns=self.config.pair_timeout_ms * 1_000_000,
            timestamp_tolerance_ns=self.config.timestamp_tolerance_ms * 1_000_000,
            max_slots=self.config.max_pair_slots,
        )
        self._h265_decoder = H265Decoder(
            self.config.expected_width,
            self.config.expected_height,
            self.config.max_h265_pending_metadata,
        )
        self._zdepth_decoder = ZdepthDecoder(
            self.config.expected_width,
            self.config.expected_height,
            self.config.max_frame_bytes,
        )

        self._h265_slot: LatestSlot[CompleteFrame] = LatestSlot()
        self._depth_slot: LatestSlot[CompleteFrame] = LatestSlot()
        self._bundle_slot: LatestSlot[DecodedBundle] = LatestSlot()
        self._publish_slot: LatestSlot[tuple] = LatestSlot()
        self._discontinuity_lock = threading.Lock()
        self._h265_discontinuity_epoch = 0
        self._state_lock = threading.Lock()
        self._started = False
        self._closed = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = None
        self._publish_timer = None
        if self._point_callback is None:
            self._publisher = self.create_publisher(PointCloud2, self.config.output_topic, qos)
            self._publish_timer = self.create_timer(0.005, self._publish_latest)
        self._metrics_timer = self.create_timer(
            self.config.metrics_interval_ms / 1000.0, self._report_metrics
        )
        self._last_metrics_ns = time.monotonic_ns()
        self._last_counts = {}

        self._threads: List[threading.Thread] = [
            threading.Thread(target=self._h265_loop, name="rgbd-h265-decode", daemon=False),
            threading.Thread(target=self._depth_loop, name="rgbd-zdepth-decode", daemon=False),
            threading.Thread(target=self._point_loop, name="rgbd-pointcloud", daemon=False),
        ]
        self._ingress = UdpIngress(
            self.config,
            frame_callback=self._route_complete_frame,
            discontinuity_callback=self._on_discontinuity,
            warning_callback=self.get_logger().warning,
        )

    def start(self) -> None:
        with self._state_lock:
            if self._closed:
                raise RuntimeError("receiver is already closed")
            if self._started:
                return
            self._started = True
        try:
            for thread in self._threads:
                thread.start()
            self._ingress.start()
        except Exception:
            self.close()
            raise
        self.get_logger().info(
            "RGB-D receiver ready: bind=%s ports=%d/%d/%d source=%d -> %s"
            % (
                self.config.bind_address,
                self.config.h265_port,
                self.config.zdepth_port,
                self.config.calibration_port,
                self.config.expected_source_id,
                self.config.output_topic if self._point_callback is None else "direct NumPy input",
            )
        )

    def _on_discontinuity(self, stream: StreamType, reason: str) -> None:
        self._metrics.increment(f"{stream.name.lower()}_discontinuity_{reason}")
        if stream == StreamType.H265:
            with self._discontinuity_lock:
                self._h265_discontinuity_epoch += 1

    def _route_complete_frame(self, frame: CompleteFrame) -> None:
        if frame.stream_type == StreamType.CALIBRATION:
            try:
                calibration = self._calibrations.accept(frame)
            except CalibrationError:
                self._metrics.increment("calibration_errors")
                return
            except Exception:
                self._metrics.increment("calibration_unexpected_errors")
                return
            self._metrics.increment("calibrations")
            self._metrics.gauge("latest_calibration_id", calibration.calibration_id)
            try:
                ready = self._pairing.ready_for_calibration(
                    calibration.source_id, calibration.calibration_id
                )
                for bundle in ready:
                    self._submit_bundle(bundle)
            except Exception:
                self._metrics.increment("calibration_pairing_errors")
            return
        try:
            if frame.stream_type == StreamType.H265:
                if self._h265_slot.put(frame):
                    self._metrics.increment("h265_input_overwrites")
            elif frame.stream_type == StreamType.ZDEPTH:
                if self._depth_slot.put(frame):
                    self._metrics.increment("zdepth_input_overwrites")
        except SlotClosed:
            self._metrics.increment("frames_after_close")

    def _submit_bundle(self, bundle: DecodedBundle) -> None:
        self._metrics.increment("paired")
        self._metrics.gauge("latest_paired_sequence", bundle.depth.bundle_sequence)
        try:
            if self._bundle_slot.put(bundle):
                self._metrics.increment("bundle_overwrites")
        except SlotClosed:
            self._metrics.increment("bundles_after_close")

    def _h265_loop(self) -> None:
        seen_overwrites = 0
        seen_discontinuity = 0
        previous_sequence = None
        while True:
            taken = self._h265_slot.wait_take()
            if taken is None:
                return
            with self._discontinuity_lock:
                discontinuity = self._h265_discontinuity_epoch
            sequence = taken.value.bundle_sequence
            sequence_gap = (
                previous_sequence is not None
                and sequence != ((previous_sequence + 1) & 0xFFFFFFFFFFFFFFFF)
            )
            previous_sequence = sequence
            if (
                taken.overwrite_count != seen_overwrites
                or discontinuity != seen_discontinuity
                or sequence_gap
            ):
                self._h265_decoder.reset_for_discontinuity()
                seen_overwrites = taken.overwrite_count
                seen_discontinuity = discontinuity
                self._metrics.increment("h265_decoder_resets")
            started = time.perf_counter_ns()
            try:
                decoded = self._h265_decoder.decode(taken.value)
            except H265DecodeError:
                self._metrics.increment("h265_decode_errors")
                continue
            except Exception:
                self._h265_decoder.reset_for_discontinuity()
                self._metrics.increment("h265_unexpected_errors")
                continue
            finally:
                self._metrics.observe_ms(
                    "h265_decode", (time.perf_counter_ns() - started) / 1_000_000.0
                )
            if decoded:
                self._metrics.observe_ms("h265_codec", self._h265_decoder.last_codec_ms)
                self._metrics.observe_ms("h265_rgb", self._h265_decoder.last_rgb_ms)
            for color in decoded:
                self._metrics.increment("h265_decoded")
                self._metrics.gauge("latest_h265_sequence", color.bundle_sequence)
                try:
                    bundle = self._pairing.push_color(color)
                    if bundle is not None:
                        self._submit_bundle(bundle)
                except Exception:
                    self._metrics.increment("h265_pairing_errors")

    def _depth_loop(self) -> None:
        while True:
            taken = self._depth_slot.wait_take()
            if taken is None:
                return
            started = time.perf_counter_ns()
            try:
                depth = self._zdepth_decoder.decode(taken.value)
            except ZdepthDecodeError:
                self._metrics.increment("zdepth_decode_errors")
                continue
            except Exception:
                self._metrics.increment("zdepth_unexpected_errors")
                continue
            finally:
                self._metrics.observe_ms(
                    "zdepth_decode", (time.perf_counter_ns() - started) / 1_000_000.0
                )
            self._metrics.increment("zdepth_decoded")
            self._metrics.gauge("latest_zdepth_sequence", depth.bundle_sequence)
            try:
                bundle = self._pairing.push_depth(depth)
                if bundle is not None:
                    self._submit_bundle(bundle)
            except Exception:
                self._metrics.increment("zdepth_pairing_errors")

    def _point_loop(self) -> None:
        while True:
            taken = self._bundle_slot.wait_take()
            if taken is None:
                return
            bundle = taken.value
            calibration = self._calibrations.get(
                bundle.depth.source_id, bundle.depth.calibration_id
            )
            if calibration is None:
                self._metrics.increment("projection_missing_calibration")
                continue
            projection_started = time.perf_counter_ns()
            try:
                points = project_color_depth(
                    bundle,
                    calibration,
                    self.config.max_output_points,
                    self.config.max_depth_mm,
                )
                if points.xyz.shape[0] == 0:
                    self._metrics.increment("empty_pointclouds")
                    continue
            except (ProjectionError, ValueError):
                self._metrics.increment("projection_errors")
                continue
            except Exception:
                self._metrics.increment("projection_unexpected_errors")
                continue
            finally:
                self._metrics.observe_ms(
                    "projection", (time.perf_counter_ns() - projection_started) / 1_000_000.0
                )
            self._metrics.gauge("latest_point_count", points.xyz.shape[0])
            frame_id = self.config.output_frame_override or calibration.color_optical_frame
            if self._point_callback is not None:
                try:
                    self._point_callback(
                        points, bundle.depth.source_timestamp_ns, frame_id
                    )
                    self._metrics.increment("direct_submitted")
                    self._metrics.gauge("latest_output_sequence", bundle.depth.bundle_sequence)
                except Exception:
                    self._metrics.increment("direct_submit_errors")
                continue
            pack_started = time.perf_counter_ns()
            try:
                message = make_pointcloud2(
                    points, bundle.depth.source_timestamp_ns, frame_id
                )
            except (ValueError, TypeError):
                self._metrics.increment("pointcloud_pack_errors")
                continue
            finally:
                self._metrics.observe_ms(
                    "pointcloud_pack", (time.perf_counter_ns() - pack_started) / 1_000_000.0
                )
            try:
                if self._publish_slot.put((bundle.depth.bundle_sequence, message)):
                    self._metrics.increment("publish_overwrites")
            except SlotClosed:
                return

    def _publish_latest(self) -> None:
        taken = self._publish_slot.take_nowait()
        if taken is None:
            return
        sequence, message = taken.value
        started = time.perf_counter_ns()
        try:
            self._publisher.publish(message)
        except Exception:
            self._metrics.increment("publish_errors")
            return
        self._metrics.observe_ms(
            "publish", (time.perf_counter_ns() - started) / 1_000_000.0
        )
        self._metrics.increment("published")
        self._metrics.gauge("latest_output_sequence", sequence)

    def _report_metrics(self) -> None:
        now_ns = time.monotonic_ns()
        interval = max((now_ns - self._last_metrics_ns) / 1_000_000_000.0, 1e-6)
        values = self._metrics.snapshot()
        udp = self._ingress.snapshot()
        pairing = self._pairing.snapshot()

        def rate(name: str) -> float:
            current = int(values.get(name, 0))
            previous = int(self._last_counts.get(name, 0))
            return (current - previous) / interval

        self.get_logger().info(
            "fps h265=%.1f depth=%.1f pair=%.1f out=%.1f | seq h/d/p/out=%s/%s/%s/%s | "
            "points=%s | udp dgrams=%s/%s/%s invalid=%d crc=%d timeout=%d | "
            "overwrite h/d/b/p=%d/%d/%d/%d | errors decode_h/depth/direct=%d/%d/%d pair_mismatch=%d | "
            "ms p95 codec/rgb/depth/xyz/pack=%.1f/%.1f/%.1f/%.1f/%.1f"
            % (
                rate("h265_decoded"),
                rate("zdepth_decoded"),
                rate("paired"),
                rate("direct_submitted" if self._point_callback is not None else "published"),
                values.get("latest_h265_sequence", "-"),
                values.get("latest_zdepth_sequence", "-"),
                values.get("latest_paired_sequence", "-"),
                values.get("latest_output_sequence", "-"),
                values.get("latest_point_count", 0),
                udp.get("h265_datagrams", 0),
                udp.get("zdepth_datagrams", 0),
                udp.get("calibration_datagrams", 0),
                sum(value for key, value in udp.items() if key.startswith("invalid_")),
                udp.get("reassembly_crc_failures", 0),
                udp.get("reassembly_timeouts", 0),
                values.get("h265_input_overwrites", 0),
                values.get("zdepth_input_overwrites", 0),
                values.get("bundle_overwrites", 0),
                values.get("publish_overwrites", 0),
                values.get("h265_decode_errors", 0),
                values.get("zdepth_decode_errors", 0),
                values.get("direct_submit_errors", 0),
                pairing.get("metadata_mismatch", 0),
                values.get("h265_codec_p95_ms", 0.0),
                values.get("h265_rgb_p95_ms", 0.0),
                values.get("zdepth_decode_p95_ms", 0.0),
                values.get("projection_p95_ms", 0.0),
                values.get("pointcloud_pack_p95_ms", 0.0),
            )
        )
        self._last_counts = values
        self._last_metrics_ns = now_ns

    def close(self) -> None:
        with self._state_lock:
            if self._closed:
                return
            self._closed = True
        alive = self._ingress.stop()
        self._h265_slot.close()
        self._depth_slot.close()
        for thread in self._threads[:2]:
            if thread.is_alive():
                thread.join(timeout=3.0)
        self._bundle_slot.close()
        if self._threads[2].is_alive():
            self._threads[2].join(timeout=3.0)
        self._publish_slot.close()
        self._h265_decoder.close()
        alive.extend(thread.name for thread in self._threads if thread.is_alive())
        if alive:
            self.get_logger().error("receiver threads did not stop cleanly: %s" % ", ".join(alive))

