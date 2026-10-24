import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .calibration import CalibrationStore
from .models import DecodedBundle, DecodedColorFrame, DecodedDepthFrame


PairKey = Tuple[int, int]


@dataclass
class PairingStats:
    paired: int = 0
    timeouts: int = 0
    evictions: int = 0
    metadata_mismatch: int = 0
    duplicate_halves: int = 0


@dataclass
class _PairSlot:
    first_seen_ns: int
    color: Optional[DecodedColorFrame] = None
    depth: Optional[DecodedDepthFrame] = None


class PairingBuffer:
    def __init__(
        self,
        calibration_store: CalibrationStore,
        timeout_ns: int,
        timestamp_tolerance_ns: int,
        max_slots: int,
    ):
        self._calibrations = calibration_store
        self._timeout_ns = timeout_ns
        self._timestamp_tolerance_ns = timestamp_tolerance_ns
        self._max_slots = max_slots
        self._slots: "OrderedDict[PairKey, _PairSlot]" = OrderedDict()
        self._stats = PairingStats()
        self._lock = threading.Lock()

    def _expire_locked(self, now_ns: int) -> None:
        keys = [
            key for key, slot in self._slots.items()
            if now_ns - slot.first_seen_ns >= self._timeout_ns
        ]
        for key in keys:
            self._slots.pop(key, None)
            self._stats.timeouts += 1

    def _try_complete(self, key: PairKey) -> Optional[DecodedBundle]:
        slot = self._slots.get(key)
        if slot is None or slot.color is None or slot.depth is None:
            return None
        color = slot.color
        depth = slot.depth
        if (
            color.source_id != depth.source_id
            or color.bundle_sequence != depth.bundle_sequence
            or color.calibration_id != depth.calibration_id
            or abs(color.source_timestamp_ns - depth.source_timestamp_ns)
            > self._timestamp_tolerance_ns
        ):
            self._slots.pop(key, None)
            self._stats.metadata_mismatch += 1
            return None
        if self._calibrations.get(color.source_id, color.calibration_id) is None:
            return None
        self._slots.pop(key, None)
        self._stats.paired += 1
        return DecodedBundle(color=color, depth=depth)

    def _push(self, value, is_color: bool, now_ns: Optional[int]) -> Optional[DecodedBundle]:
        if now_ns is None:
            now_ns = time.monotonic_ns()
        key = (value.source_id, value.bundle_sequence)
        with self._lock:
            self._expire_locked(now_ns)
            slot = self._slots.get(key)
            if slot is None:
                while len(self._slots) >= self._max_slots:
                    self._slots.popitem(last=False)
                    self._stats.evictions += 1
                slot = _PairSlot(first_seen_ns=now_ns)
                self._slots[key] = slot
            else:
                self._slots.move_to_end(key)
            if is_color:
                if slot.color is not None:
                    self._stats.duplicate_halves += 1
                slot.color = value
            else:
                if slot.depth is not None:
                    self._stats.duplicate_halves += 1
                slot.depth = value
            return self._try_complete(key)

    def push_color(
        self, frame: DecodedColorFrame, now_ns: Optional[int] = None
    ) -> Optional[DecodedBundle]:
        return self._push(frame, True, now_ns)

    def push_depth(
        self, frame: DecodedDepthFrame, now_ns: Optional[int] = None
    ) -> Optional[DecodedBundle]:
        return self._push(frame, False, now_ns)

    def ready_for_calibration(self, source_id: int, calibration_id: int) -> List[DecodedBundle]:
        ready: List[DecodedBundle] = []
        with self._lock:
            for key in list(self._slots):
                slot = self._slots[key]
                if (
                    slot.color is not None
                    and slot.depth is not None
                    and slot.color.source_id == source_id
                    and slot.color.calibration_id == calibration_id
                ):
                    bundle = self._try_complete(key)
                    if bundle is not None:
                        ready.append(bundle)
        return ready

    def snapshot(self) -> dict:
        with self._lock:
            result = dict(vars(self._stats))
            result["pair_slots"] = len(self._slots)
            return result
