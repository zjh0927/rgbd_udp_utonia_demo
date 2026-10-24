from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Tuple


class StreamType(IntEnum):
    H265 = 1
    ZDEPTH = 2
    CALIBRATION = 3


FLAG_KEYFRAME = 1 << 0
FLAG_CONFIG = 1 << 1


@dataclass(frozen=True)
class PacketHeader:
    magic: int
    version: int
    stream_type: StreamType
    flags: int
    source_id: int
    calibration_id: int
    bundle_sequence: int
    source_timestamp_ns: int
    send_timestamp_ns: int
    frame_size: int
    frame_crc32: int
    fragment_index: int
    fragment_count: int
    fragment_payload_size: int
    reserved: int


@dataclass(frozen=True)
class Datagram:
    header: PacketHeader
    payload: bytes


@dataclass(frozen=True)
class CompleteFrame:
    source_id: int
    stream_type: StreamType
    flags: int
    calibration_id: int
    bundle_sequence: int
    source_timestamp_ns: int
    send_timestamp_ns: int
    payload: bytes
    completed_monotonic_ns: int


@dataclass(frozen=True)
class Calibration:
    source_id: int
    calibration_id: int
    width: int
    height: int
    color_encoding: str
    depth_encoding: str
    distortion_model: str
    d: Tuple[float, ...]
    k: Tuple[float, ...]
    r: Tuple[float, ...]
    p: Tuple[float, ...]
    depth_scale_m: float
    color_optical_frame: str
    depth_aligned_frame: str


@dataclass(frozen=True)
class DecodedColorFrame:
    source_id: int
    calibration_id: int
    bundle_sequence: int
    source_timestamp_ns: int
    pixels: Any


@dataclass(frozen=True)
class DecodedDepthFrame:
    source_id: int
    calibration_id: int
    bundle_sequence: int
    source_timestamp_ns: int
    millimeters: Any


@dataclass(frozen=True)
class DecodedBundle:
    color: DecodedColorFrame
    depth: DecodedDepthFrame


@dataclass(frozen=True)
class ColoredPoints:
    xyz: Any
    rgb: Any
