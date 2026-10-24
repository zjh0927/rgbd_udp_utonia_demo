import numpy as np

from .models import (
    FLAG_KEYFRAME,
    CompleteFrame,
    DecodedDepthFrame,
    StreamType,
)


class ZdepthDecodeError(RuntimeError):
    pass


class ZdepthDecoder:
    def __init__(self, max_width: int, max_height: int, max_frame_bytes: int):
        try:
            from ._zdepth import Decoder
        except ImportError as exc:
            raise ZdepthDecodeError(
                "无法导入 Zdepth 扩展；请先用 colcon 构建并 source install/setup.bash"
            ) from exc
        self._expected_width = max_width
        self._expected_height = max_height
        self._decoder = Decoder(
            max_width,
            max_height,
            max_width * max_height,
            max_frame_bytes,
        )

    def decode(self, frame: CompleteFrame) -> DecodedDepthFrame:
        if frame.stream_type != StreamType.ZDEPTH:
            raise ZdepthDecodeError("received a non-Zdepth frame")
        if not frame.flags & FLAG_KEYFRAME:
            raise ZdepthDecodeError("v1 receiver requires all Zdepth frames to be keyframes")
        try:
            millimeters = self._decoder.decode(frame.payload)
        except RuntimeError as exc:
            raise ZdepthDecodeError(str(exc)) from exc
        if millimeters.dtype != np.uint16 or millimeters.ndim != 2:
            raise ZdepthDecodeError("Zdepth extension returned an invalid array")
        if millimeters.shape != (self._expected_height, self._expected_width):
            raise ZdepthDecodeError(
                f"unexpected depth dimensions {millimeters.shape}; expected "
                f"{(self._expected_height, self._expected_width)}"
            )
        if not millimeters.flags.c_contiguous:
            millimeters = np.ascontiguousarray(millimeters)
        return DecodedDepthFrame(
            source_id=frame.source_id,
            calibration_id=frame.calibration_id,
            bundle_sequence=frame.bundle_sequence,
            source_timestamp_ns=frame.source_timestamp_ns,
            millimeters=millimeters,
        )
