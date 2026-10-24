import json
import math
import threading
from collections import OrderedDict, defaultdict
from typing import Dict, Optional, Sequence, Tuple

from .models import FLAG_CONFIG, Calibration, CompleteFrame, StreamType


class CalibrationError(ValueError):
    pass


def _number_tuple(document: dict, name: str, length: Optional[int] = None) -> Tuple[float, ...]:
    value = document.get(name)
    if not isinstance(value, list) or (length is not None and len(value) != length):
        raise CalibrationError(f"{name} has an invalid length or type")
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
            raise CalibrationError(f"{name} contains a non-finite/non-numeric value")
        result.append(float(item))
    return tuple(result)


def _positive_int(document: dict, name: str) -> int:
    value = document.get(name)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise CalibrationError(f"{name} must be a positive integer")
    return value


class CalibrationStore:
    def __init__(self, max_generations_per_source: int, expected_width: int, expected_height: int):
        self._max_generations = max_generations_per_source
        self._expected_width = expected_width
        self._expected_height = expected_height
        self._by_source: Dict[int, "OrderedDict[int, Calibration]"] = defaultdict(OrderedDict)
        self._lock = threading.Lock()

    def parse(self, frame: CompleteFrame) -> Calibration:
        if frame.stream_type != StreamType.CALIBRATION or not frame.flags & FLAG_CONFIG:
            raise CalibrationError("frame is not a calibration CONFIG frame")
        if frame.bundle_sequence != 0:
            raise CalibrationError("calibration bundle_sequence must be zero")
        try:
            document = json.loads(frame.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise CalibrationError(f"invalid calibration JSON: {exc}") from exc
        if not isinstance(document, dict):
            raise CalibrationError("calibration JSON root must be an object")
        protocol_version = document.get("protocol_version")
        if (
            document.get("type") != "rgbd_calibration"
            or isinstance(protocol_version, bool)
            or protocol_version != 1
        ):
            raise CalibrationError("calibration type or protocol version is incompatible")
        calibration_id = _positive_int(document, "calibration_id")
        if calibration_id != frame.calibration_id:
            raise CalibrationError("header and JSON calibration_id differ")
        width = _positive_int(document, "width")
        height = _positive_int(document, "height")
        if (width, height) != (self._expected_width, self._expected_height):
            raise CalibrationError(
                f"unexpected calibration dimensions {width}x{height}; expected "
                f"{self._expected_width}x{self._expected_height}"
            )
        color_encoding = document.get("color_encoding")
        if color_encoding not in ("rgb8", "bgr8"):
            raise CalibrationError("color_encoding must be rgb8 or bgr8")
        if document.get("depth_encoding") not in ("16UC1", "mono16"):
            raise CalibrationError("depth_encoding must be 16UC1 or mono16")
        k = _number_tuple(document, "k", 9)
        if k[0] <= 0.0 or k[4] <= 0.0:
            raise CalibrationError("calibration fx and fy must be positive")
        depth_scale = document.get("depth_scale_m")
        if (
            isinstance(depth_scale, bool)
            or not isinstance(depth_scale, (int, float))
            or not math.isfinite(depth_scale)
            or depth_scale <= 0.0
        ):
            raise CalibrationError("depth_scale_m must be finite and positive")
        color_frame = document.get("color_optical_frame")
        depth_frame = document.get("depth_aligned_frame")
        if not isinstance(color_frame, str) or not color_frame:
            raise CalibrationError("color_optical_frame must not be empty")
        if not isinstance(depth_frame, str) or not depth_frame:
            raise CalibrationError("depth_aligned_frame must not be empty")
        zdepth = document.get("zdepth")
        if not isinstance(zdepth, dict) or zdepth.get("encode_mode") != "not_quantized_8191mm":
            raise CalibrationError("unsupported Zdepth encode mode")
        if zdepth.get("all_keyframes") is not True:
            raise CalibrationError("v1 receiver requires all-keyframe Zdepth")
        h265 = document.get("h265")
        if not isinstance(h265, dict):
            raise CalibrationError("missing h265 configuration")
        if h265.get("stream_format") != "byte-stream" or h265.get("alignment") != "au":
            raise CalibrationError("H.265 must use byte-stream AU alignment")
        if isinstance(h265.get("b_frames"), bool) or h265.get("b_frames") != 0:
            raise CalibrationError("v1 metadata matching requires H.265 b_frames=0")
        for name in ("bitrate", "gop", "idr_interval", "fps"):
            _positive_int(h265, name)
        if h265.get("rate_control") != "cbr_if_supported":
            raise CalibrationError("unsupported H.265 rate control description")
        for name in ("binning_x", "binning_y"):
            value = document.get(name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise CalibrationError(f"{name} must be a non-negative integer")
        roi = document.get("roi")
        if not isinstance(roi, dict):
            raise CalibrationError("roi must be an object")
        for name in ("x_offset", "y_offset", "height", "width"):
            value = roi.get(name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise CalibrationError(f"roi.{name} must be a non-negative integer")
        if not isinstance(roi.get("do_rectify"), bool):
            raise CalibrationError("roi.do_rectify must be boolean")

        return Calibration(
            source_id=frame.source_id,
            calibration_id=calibration_id,
            width=width,
            height=height,
            color_encoding=color_encoding,
            depth_encoding=document["depth_encoding"],
            distortion_model=str(document.get("distortion_model", "")),
            d=_number_tuple(document, "d"),
            k=k,
            r=_number_tuple(document, "r", 9),
            p=_number_tuple(document, "p", 12),
            depth_scale_m=float(depth_scale),
            color_optical_frame=color_frame,
            depth_aligned_frame=depth_frame,
        )

    def accept(self, frame: CompleteFrame) -> Calibration:
        calibration = self.parse(frame)
        with self._lock:
            versions = self._by_source[calibration.source_id]
            versions[calibration.calibration_id] = calibration
            versions.move_to_end(calibration.calibration_id)
            while len(versions) > self._max_generations:
                versions.popitem(last=False)
        return calibration

    def get(self, source_id: int, calibration_id: int) -> Optional[Calibration]:
        with self._lock:
            return self._by_source.get(source_id, {}).get(calibration_id)
