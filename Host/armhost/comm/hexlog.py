"""原始收发字节的 hex 记录：内存环形缓冲（给 UI 看）+ 可选 logger 输出。"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass

raw_log = logging.getLogger("armhost.raw")


def hexdump(data: bytes, sep: str = " ") -> str:
    return data.hex(sep).upper() if data else ""


@dataclass(frozen=True, slots=True)
class HexRecord:
    host_time_ns: int
    direction: str        # "RX" / "TX"
    data: bytes

    def format(self) -> str:
        return f"{self.host_time_ns / 1e9:12.6f} {self.direction} [{len(self.data):3d}] {hexdump(self.data)}"


class HexLog:
    def __init__(self, capacity: int = 2000) -> None:
        self._buf: deque[HexRecord] = deque(maxlen=capacity)
        self._lock = threading.Lock()
        self.enabled = False

    def record(self, direction: str, data: bytes) -> None:
        if not self.enabled:
            return
        rec = HexRecord(time.monotonic_ns(), direction, bytes(data))
        with self._lock:
            self._buf.append(rec)
        if raw_log.isEnabledFor(logging.DEBUG):
            raw_log.debug(rec.format())

    def snapshot(self, last: int | None = None) -> list[HexRecord]:
        with self._lock:
            items = list(self._buf)
        return items[-last:] if last else items

    def clear(self) -> None:
        with self._lock:
            self._buf.clear()
