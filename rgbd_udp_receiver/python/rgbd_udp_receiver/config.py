from dataclasses import dataclass, fields
from typing import Optional


@dataclass(frozen=True)
class ReceiverConfig:
    bind_address: str = "0.0.0.0"
    h265_port: int = 5000
    zdepth_port: int = 5001
    calibration_port: int = 5002
    expected_source_id: int = 1
    reassembly_timeout_ms: int = 100
    pair_timeout_ms: int = 100
    timestamp_tolerance_ms: int = 10
    socket_timeout_ms: int = 100
    max_datagram_bytes: int = 1200
    max_frame_bytes: int = 16 * 1024 * 1024
    max_fragments: int = 4096
    max_inflight_frames: int = 64
    max_inflight_bytes: int = 64 * 1024 * 1024
    socket_receive_buffer_bytes: int = 8 * 1024 * 1024
    max_pair_slots: int = 64
    max_h265_pending_metadata: int = 32
    max_calibration_generations: int = 4
    max_output_points: int = 50000
    expected_width: int = 640
    expected_height: int = 480
    max_depth_mm: int = 8190
    output_topic: str = "/rgbd/color_points"
    output_frame_override: str = ""
    preview_fps: int = 0
    metrics_interval_ms: int = 1000

    def validation_error(self) -> Optional[str]:
        if not self.bind_address.strip():
            return "bind_address must not be empty"
        ports = (self.h265_port, self.zdepth_port, self.calibration_port)
        if any(port < 1 or port > 65535 for port in ports):
            return "ports must be in [1, 65535]"
        if len(set(ports)) != len(ports):
            return "UDP ports must be distinct"
        if not 0 <= self.expected_source_id <= 0xFFFFFFFF:
            return "expected_source_id must fit uint32"
        if not 10 <= self.reassembly_timeout_ms <= 5000:
            return "reassembly_timeout_ms must be in [10, 5000]"
        if not 10 <= self.pair_timeout_ms <= 5000:
            return "pair_timeout_ms must be in [10, 5000]"
        if not 0 <= self.timestamp_tolerance_ms <= 1000:
            return "timestamp_tolerance_ms must be in [0, 1000]"
        if not 10 <= self.socket_timeout_ms <= 1000:
            return "socket_timeout_ms must be in [10, 1000]"
        if not 57 <= self.max_datagram_bytes <= 65507:
            return "max_datagram_bytes must be in [57, 65507]"
        if not 1 <= self.max_frame_bytes <= 256 * 1024 * 1024:
            return "max_frame_bytes must be in [1, 256 MiB]"
        if not 1 <= self.max_fragments <= 65535:
            return "max_fragments must be in [1, 65535]"
        if not 1 <= self.max_inflight_frames <= 4096:
            return "max_inflight_frames must be in [1, 4096]"
        if not self.max_frame_bytes <= self.max_inflight_bytes <= 1024 * 1024 * 1024:
            return "max_inflight_bytes must cover one frame and be at most 1 GiB"
        if not 64 * 1024 <= self.socket_receive_buffer_bytes <= 256 * 1024 * 1024:
            return "socket_receive_buffer_bytes must be in [64 KiB, 256 MiB]"
        if not 1 <= self.max_pair_slots <= 4096:
            return "max_pair_slots must be in [1, 4096]"
        if not 1 <= self.max_h265_pending_metadata <= 1024:
            return "max_h265_pending_metadata must be in [1, 1024]"
        if not 1 <= self.max_calibration_generations <= 64:
            return "max_calibration_generations must be in [1, 64]"
        if not 1 <= self.max_output_points <= 307200:
            return "max_output_points must be in [1, 307200]"
        if self.expected_width <= 0 or self.expected_height <= 0:
            return "expected dimensions must be positive"
        if self.expected_width * self.expected_height > 16 * 1024 * 1024:
            return "expected image is unreasonably large"
        if not 1 <= self.max_depth_mm <= 8190:
            return "max_depth_mm must be in [1, 8190]"
        if not self.output_topic.startswith("/"):
            return "output_topic must be absolute"
        if not 0 <= self.preview_fps <= 30:
            return "preview_fps must be in [0, 30]"
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
