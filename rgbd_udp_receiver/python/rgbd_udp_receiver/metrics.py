import math
import threading
from collections import defaultdict, deque
from typing import Deque, Dict


class Metrics:
    def __init__(self):
        self._lock = threading.Lock()
        self._counters: Dict[str, int] = defaultdict(int)
        self._gauges: Dict[str, int] = {}
        self._samples: Dict[str, Deque[float]] = defaultdict(lambda: deque(maxlen=256))

    def increment(self, name: str, amount: int = 1) -> None:
        with self._lock:
            self._counters[name] += amount

    def gauge(self, name: str, value: int) -> None:
        with self._lock:
            self._gauges[name] = value

    def observe_ms(self, name: str, elapsed_ms: float) -> None:
        with self._lock:
            self._samples[name].append(float(elapsed_ms))

    def snapshot(self) -> dict:
        with self._lock:
            result = dict(self._counters)
            result.update(self._gauges)
            for name, values in self._samples.items():
                if not values:
                    continue
                ordered = sorted(values)
                rank = max(0, math.ceil(0.95 * len(ordered)) - 1)
                result[f"{name}_avg_ms"] = sum(ordered) / len(ordered)
                result[f"{name}_p95_ms"] = ordered[rank]
            return result
