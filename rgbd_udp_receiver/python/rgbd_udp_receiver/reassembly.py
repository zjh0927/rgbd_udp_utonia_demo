import threading
import zlib
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, Tuple

from .models import CompleteFrame, Datagram, PacketHeader, StreamType


FrameKey = Tuple[int, StreamType, int]
DropCallback = Callable[[StreamType, str], None]


@dataclass
class ReassemblyStats:
    completed: int = 0
    duplicate_fragments: int = 0
    metadata_conflicts: int = 0
    duplicate_conflicts: int = 0
    crc_failures: int = 0
    length_failures: int = 0
    timeouts: int = 0
    evictions: int = 0
    stale_fragments: int = 0


@dataclass
class _Slot:
    header: PacketHeader
    first_seen_ns: int
    fragments: Dict[int, bytes] = field(default_factory=dict)
    received_bytes: int = 0


class ReassemblyManager:
    def __init__(
        self,
        timeout_ns: int,
        max_inflight_frames: int,
        max_inflight_bytes: int,
        drop_callback: Optional[DropCallback] = None,
    ):
        self._timeout_ns = timeout_ns
        self._max_frames = max_inflight_frames
        self._max_bytes = max_inflight_bytes
        self._drop_callback = drop_callback
        self._slots: "OrderedDict[FrameKey, _Slot]" = OrderedDict()
        self._recently_completed: "OrderedDict[FrameKey, int]" = OrderedDict()
        self._newest_sequence: Dict[Tuple[int, StreamType], Tuple[int, int]] = {}
        self._inflight_bytes = 0
        self._lock = threading.Lock()
        self._stats = ReassemblyStats()

    @staticmethod
    def _key(header: PacketHeader) -> FrameKey:
        return (header.source_id, header.stream_type, header.bundle_sequence)

    @staticmethod
    def _same_frame(a: PacketHeader, b: PacketHeader) -> bool:
        return (
            a.version == b.version
            and a.stream_type == b.stream_type
            and a.flags == b.flags
            and a.source_id == b.source_id
            and a.calibration_id == b.calibration_id
            and a.bundle_sequence == b.bundle_sequence
            and a.source_timestamp_ns == b.source_timestamp_ns
            and a.send_timestamp_ns == b.send_timestamp_ns
            and a.frame_size == b.frame_size
            and a.frame_crc32 == b.frame_crc32
            and a.fragment_count == b.fragment_count
        )

    def _notify_drop(self, stream: StreamType, reason: str) -> None:
        if self._drop_callback is not None:
            self._drop_callback(stream, reason)

    def _remove(self, key: FrameKey, reason: Optional[str] = None) -> None:
        slot = self._slots.pop(key, None)
        if slot is None:
            return
        self._inflight_bytes -= slot.received_bytes
        if reason is not None:
            self._notify_drop(slot.header.stream_type, reason)

    def _expire_locked(self, now_ns: int) -> int:
        expired = [
            key for key, slot in self._slots.items()
            if now_ns - slot.first_seen_ns >= self._timeout_ns
        ]
        for key in expired:
            self._stats.timeouts += 1
            self._remove(key, "timeout")
        completed_expired = [
            key for key, completed_ns in self._recently_completed.items()
            if now_ns - completed_ns >= self._timeout_ns
        ]
        for key in completed_expired:
            self._recently_completed.pop(key, None)
        return len(expired)

    def expire(self, now_ns: int) -> int:
        with self._lock:
            return self._expire_locked(now_ns)

    def _make_room(self, incoming_bytes: int) -> None:
        while self._slots and (
            len(self._slots) >= self._max_frames
            or self._inflight_bytes + incoming_bytes > self._max_bytes
        ):
            oldest = next(iter(self._slots))
            self._stats.evictions += 1
            self._remove(oldest, "evicted")

    def push(self, datagram: Datagram, now_ns: int) -> Optional[CompleteFrame]:
        header = datagram.header
        key = self._key(header)
        with self._lock:
            self._expire_locked(now_ns)
            if key in self._recently_completed:
                self._stats.duplicate_fragments += 1
                return None
            if header.stream_type != StreamType.CALIBRATION:
                stream_key = (header.source_id, header.stream_type)
                newest = self._newest_sequence.get(stream_key)
                if newest is None or now_ns - newest[1] >= self._timeout_ns:
                    self._newest_sequence[stream_key] = (header.bundle_sequence, now_ns)
                elif header.bundle_sequence < newest[0]:
                    self._stats.stale_fragments += 1
                    return None
                elif header.bundle_sequence > newest[0]:
                    self._newest_sequence[stream_key] = (header.bundle_sequence, now_ns)
                    old_keys = [
                        candidate
                        for candidate in self._slots
                        if candidate[0] == header.source_id
                        and candidate[1] == header.stream_type
                        and candidate[2] < header.bundle_sequence
                    ]
                    for old_key in old_keys:
                        self._stats.evictions += 1
                        self._remove(old_key, "newer_sequence")
                else:
                    self._newest_sequence[stream_key] = (newest[0], now_ns)
            slot = self._slots.get(key)
            if slot is None:
                if len(datagram.payload) > self._max_bytes:
                    self._stats.evictions += 1
                    self._notify_drop(header.stream_type, "memory_limit")
                    return None
                self._make_room(len(datagram.payload))
                slot = _Slot(header=header, first_seen_ns=now_ns)
                self._slots[key] = slot
            elif not self._same_frame(slot.header, header):
                self._stats.metadata_conflicts += 1
                self._remove(key, "metadata_conflict")
                return None

            previous = slot.fragments.get(header.fragment_index)
            if previous is not None:
                if previous == datagram.payload:
                    self._stats.duplicate_fragments += 1
                    return None
                self._stats.duplicate_conflicts += 1
                self._remove(key, "duplicate_conflict")
                return None

            extra = len(datagram.payload)
            if self._inflight_bytes + extra > self._max_bytes:
                self._stats.evictions += 1
                self._remove(key, "memory_limit")
                return None
            slot.fragments[header.fragment_index] = datagram.payload
            slot.received_bytes += extra
            self._inflight_bytes += extra
            self._slots.move_to_end(key)

            if len(slot.fragments) != header.fragment_count:
                return None
            payload = b"".join(slot.fragments[index] for index in range(header.fragment_count))
            self._remove(key)
            if len(payload) != header.frame_size:
                self._stats.length_failures += 1
                self._notify_drop(header.stream_type, "length")
                return None
            if zlib.crc32(payload) & 0xFFFFFFFF != header.frame_crc32:
                self._stats.crc_failures += 1
                self._notify_drop(header.stream_type, "crc")
                return None
            self._stats.completed += 1
            self._recently_completed[key] = now_ns
            while len(self._recently_completed) > self._max_frames * 2:
                self._recently_completed.popitem(last=False)
            return CompleteFrame(
                source_id=header.source_id,
                stream_type=header.stream_type,
                flags=header.flags,
                calibration_id=header.calibration_id,
                bundle_sequence=header.bundle_sequence,
                source_timestamp_ns=header.source_timestamp_ns,
                send_timestamp_ns=header.send_timestamp_ns,
                payload=payload,
                completed_monotonic_ns=now_ns,
            )

    def snapshot(self) -> dict:
        with self._lock:
            result = dict(vars(self._stats))
            result["inflight_frames"] = len(self._slots)
            result["inflight_bytes"] = self._inflight_bytes
            return result
