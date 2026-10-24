import struct
from dataclasses import dataclass
from typing import Optional

from .models import Datagram, PacketHeader, StreamType


MAGIC = 0x52474244
VERSION = 1
HEADER = struct.Struct("!IBBHIIQQQIIHHHH")
HEADER_SIZE = HEADER.size
KNOWN_FLAGS = 0x03


@dataclass(frozen=True)
class ProtocolLimits:
    expected_source_id: int
    max_datagram_bytes: int
    max_frame_bytes: int
    max_fragments: int


class PacketValidationError(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def _reject(code: str, message: str):
    raise PacketValidationError(code, message)


def parse_datagram(
    data: bytes,
    expected_stream: Optional[StreamType],
    limits: ProtocolLimits,
) -> Datagram:
    size = len(data)
    if size < HEADER_SIZE:
        _reject("short_datagram", "datagram is shorter than the v1 header")
    if size > limits.max_datagram_bytes:
        _reject("oversize_datagram", "datagram exceeds configured maximum")

    raw = HEADER.unpack_from(data)
    if raw[0] != MAGIC:
        _reject("bad_magic", "unexpected protocol magic")
    if raw[1] != VERSION:
        _reject("bad_version", "unsupported protocol version")
    try:
        stream_type = StreamType(raw[2])
    except ValueError:
        _reject("bad_stream", "unknown stream type")
    if expected_stream is not None and stream_type != expected_stream:
        _reject("wrong_port", "stream arrived on the wrong UDP port")
    if raw[3] & ~KNOWN_FLAGS:
        _reject("bad_flags", "unknown protocol flags are set")
    if raw[4] != limits.expected_source_id:
        _reject("wrong_source", "source_id does not match configuration")
    if raw[14] != 0:
        _reject("reserved_nonzero", "reserved field must be zero")

    frame_size = raw[9]
    fragment_index = raw[11]
    fragment_count = raw[12]
    payload_size = raw[13]
    actual_payload_size = size - HEADER_SIZE
    if payload_size != actual_payload_size:
        _reject("payload_size", "declared fragment payload length does not match datagram")
    if payload_size == 0:
        _reject("empty_fragment", "empty fragments are not allowed")
    if frame_size == 0 or frame_size > limits.max_frame_bytes:
        _reject("frame_size", "frame size is zero or exceeds configured maximum")
    if fragment_count == 0 or fragment_count > limits.max_fragments:
        _reject("fragment_count", "fragment count is zero or exceeds configured maximum")
    if fragment_index >= fragment_count:
        _reject("fragment_index", "fragment index is outside fragment count")

    capacity = limits.max_datagram_bytes - HEADER_SIZE
    if frame_size > fragment_count * capacity:
        _reject("impossible_capacity", "fragments cannot contain declared frame size")
    if frame_size < fragment_count or payload_size > frame_size:
        _reject("fragment_bounds", "fragment layout cannot match declared frame size")

    header = PacketHeader(
        magic=raw[0],
        version=raw[1],
        stream_type=stream_type,
        flags=raw[3],
        source_id=raw[4],
        calibration_id=raw[5],
        bundle_sequence=raw[6],
        source_timestamp_ns=raw[7],
        send_timestamp_ns=raw[8],
        frame_size=frame_size,
        frame_crc32=raw[10],
        fragment_index=fragment_index,
        fragment_count=fragment_count,
        fragment_payload_size=payload_size,
        reserved=raw[14],
    )
    return Datagram(header=header, payload=bytes(data[HEADER_SIZE:]))
