import threading
from dataclasses import dataclass
from typing import Generic, Optional, TypeVar


T = TypeVar("T")


class SlotClosed(RuntimeError):
    pass


@dataclass(frozen=True)
class Taken(Generic[T]):
    value: T
    overwrite_count: int


class LatestSlot(Generic[T]):
    """A blocking, closeable container that retains at most one value."""

    def __init__(self):
        self._condition = threading.Condition()
        self._value: Optional[T] = None
        self._has_value = False
        self._closed = False
        self._overwrites = 0

    def put(self, value: T) -> bool:
        with self._condition:
            if self._closed:
                raise SlotClosed("latest-value slot is closed")
            overwritten = self._has_value
            if overwritten:
                self._overwrites += 1
            self._value = value
            self._has_value = True
            self._condition.notify()
            return overwritten

    def wait_take(self) -> Optional[Taken[T]]:
        with self._condition:
            while not self._has_value and not self._closed:
                self._condition.wait()
            if not self._has_value:
                return None
            value = self._value
            self._value = None
            self._has_value = False
            return Taken(value=value, overwrite_count=self._overwrites)

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    @property
    def overwrite_count(self) -> int:
        with self._condition:
            return self._overwrites
