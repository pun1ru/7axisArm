"""流式帧解析器：支持不完整帧、连续多帧、错误帧与重新同步。

规则（与 firmware_ref/arm_proto.c 一致）：
1. 在缓冲里找 A5 5A；之前的字节计为 discarded。
2. 凑够 9 字节后校验 HCRC；失败 -> header_error，丢 1 字节重新找同步。
   HCRC 保证了 LEN 可信，避免一个被破坏的 LEN 让解析器空等。
3. LEN > MAX_PAYLOAD -> length_error，丢 1 字节。
4. 凑齐整帧后校验 CRC16；失败 -> crc_error，丢 1 字节（不能整帧跳过，
   因为真正的帧头可能就在这段坏数据里面）。
5. major 版本不同 -> version_error，整帧跳过（帧本身是完整的）。
6. 半帧超过 stale_timeout 没收齐 -> timeout_drop，丢弃半帧。
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from typing import Callable

from .crc import crc8, crc16
from .frame import HEADER_SIZE, MAX_PAYLOAD, PROTO_MAJOR, SOF0, SOF1, TRAILER_SIZE, Frame


@dataclass
class ParserStats:
    frames_ok: int = 0
    bytes_in: int = 0
    bytes_discarded: int = 0
    header_errors: int = 0
    length_errors: int = 0
    crc_errors: int = 0
    version_errors: int = 0
    timeout_drops: int = 0

    def as_dict(self) -> dict[str, int]:
        return dict(self.__dict__)


@dataclass
class FrameParser:
    stale_timeout_s: float = 0.05
    max_buffer: int = 8192
    clock: Callable[[], float] = time.monotonic
    stats: ParserStats = field(default_factory=ParserStats)
    _buf: bytearray = field(default_factory=bytearray)
    _partial_since: float | None = None

    def reset(self) -> None:
        self._buf.clear()
        self._partial_since = None

    def feed(self, data: bytes) -> list[Frame]:
        now = self.clock()
        if (self._partial_since is not None and self._buf
                and now - self._partial_since > self.stale_timeout_s):
            self.stats.timeout_drops += 1
            self.stats.bytes_discarded += len(self._buf)
            self._buf.clear()
        self.stats.bytes_in += len(data)
        self._buf += data
        if len(self._buf) > self.max_buffer:          # 理论上不会到，防御
            drop = len(self._buf) - self.max_buffer
            del self._buf[:drop]
            self.stats.bytes_discarded += drop
        frames = self._drain()
        self._partial_since = now if self._buf else None
        return frames

    def _drop(self, n: int) -> None:
        del self._buf[:n]
        self.stats.bytes_discarded += n

    def _drain(self) -> list[Frame]:
        out: list[Frame] = []
        buf = self._buf
        while True:
            # 1. 找同步
            idx = buf.find(bytes((SOF0, SOF1)))
            if idx < 0:
                # 保留最后一个可能是 SOF0 的字节
                keep = 1 if buf and buf[-1] == SOF0 else 0
                if len(buf) > keep:
                    self._drop(len(buf) - keep)
                return out
            if idx:
                self._drop(idx)
            if len(buf) < HEADER_SIZE:
                return out
            # 2. 帧头
            if crc8(buf[:8]) != buf[8]:
                self.stats.header_errors += 1
                self._drop(1)
                continue
            length = buf[6]
            if length > MAX_PAYLOAD:
                self.stats.length_errors += 1
                self._drop(1)
                continue
            total = HEADER_SIZE + length + TRAILER_SIZE
            if len(buf) < total:
                return out
            # 3. 整帧 CRC
            (rx_crc,) = struct.unpack_from("<H", buf, total - 2)
            if crc16(buf[:total - 2]) != rx_crc:
                self.stats.crc_errors += 1
                self._drop(1)
                continue
            version = buf[2]
            if version >> 4 != PROTO_MAJOR:
                self.stats.version_errors += 1
                self._drop(total)
                continue
            (seq,) = struct.unpack_from("<H", buf, 4)
            out.append(Frame(version=version, msg_type=buf[3], seq=seq,
                             flags=buf[7], payload=bytes(buf[HEADER_SIZE:HEADER_SIZE + length])))
            del buf[:total]
            self.stats.frames_ok += 1
