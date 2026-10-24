from dataclasses import dataclass, fields
from typing import Optional


@dataclass(frozen=True)
class InferenceConfig:
    input_topic: str = "/rgbd/color_points"
    output_topic: str = "/utonia/pca_points"
    output_frame_id: str = ""
    utonia_model: str = "utonia_tiny"
    utonia_source_path: str = "~/utonia_ws/Utonia"
    checkpoint_path: str = "~/utonia_ws/Utonia/ckpt/utonia_tiny.pth"
    utonia_scale: float = 0.5
    max_model_points: int = 50000
    min_model_points: int = 256
    oom_min_points: int = 4096
    oom_max_consecutive: int = 3
    warmup_points: int = 4096
    random_seed: int = 37
    pca_profile_path: str = (
        "~/utonia_ws/Utonia/ckpt/utonia_tiny_pca_profile.npz"
    )
    pca_calibration_frames: int = 20
    pca_max_samples: int = 65536
    metrics_interval_ms: int = 1000

    def validation_error(self) -> Optional[str]:
        if not self.input_topic.startswith("/"):
            return "input_topic must be absolute"
        if not self.output_topic.startswith("/"):
            return "output_topic must be absolute"
        if self.input_topic == self.output_topic:
            return "input_topic and output_topic must differ"
        if self.utonia_model != "utonia_tiny":
            return "utonia_model must be utonia_tiny in the first release"
        if not self.utonia_source_path.strip():
            return "utonia_source_path must not be empty"
        if not self.checkpoint_path.strip():
            return "checkpoint_path must not be empty"
        if not 0.01 <= self.utonia_scale <= 10.0:
            return "utonia_scale must be in [0.01, 10.0]"
        if not 256 <= self.max_model_points <= 307200:
            return "max_model_points must be in [256, 307200]"
        if not 16 <= self.min_model_points <= self.max_model_points:
            return "min_model_points must be in [16, max_model_points]"
        if not self.min_model_points <= self.oom_min_points <= self.max_model_points:
            return "oom_min_points must be in [min_model_points, max_model_points]"
        if not 1 <= self.oom_max_consecutive <= 10:
            return "oom_max_consecutive must be in [1, 10]"
        if not self.min_model_points <= self.warmup_points <= self.max_model_points:
            return "warmup_points must be in [min_model_points, max_model_points]"
        if not 0 <= self.random_seed <= 0xFFFFFFFF:
            return "random_seed must fit uint32"
        if not self.pca_profile_path.strip():
            return "pca_profile_path must not be empty"
        if not 2 <= self.pca_calibration_frames <= 1000:
            return "pca_calibration_frames must be in [2, 1000]"
        if not 1024 <= self.pca_max_samples <= 1_000_000:
            return "pca_max_samples must be in [1024, 1000000]"
        if not 100 <= self.metrics_interval_ms <= 60000:
            return "metrics_interval_ms must be in [100, 60000]"
        return None

    @classmethod
    def from_node(cls, node):
        defaults = cls()
        values = {}
        for field in fields(defaults):
            default = getattr(defaults, field.name)
            values[field.name] = node.declare_parameter(field.name, default).value
        config = cls(**values)
        error = config.validation_error()
        if error:
            raise ValueError(error)
        return config
