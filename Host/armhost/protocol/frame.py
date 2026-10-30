"""帧格式（全部小端）

偏移  长度  字段
 0    1    SOF0   = 0xA5
 1    1    SOF1   = 0x5A
 2    1    VER    高 4 位 major，低 4 位 minor（当前 0x10 = v1.0）
 3    1    TYPE   MsgType
 4    2    SEQ    u16，发送方每帧 +1，回绕
 6    1    LEN    负载长度 0..MAX_PAYLOAD
 7    1    FLAGS  FrameFlag
 8    1    HCRC   CRC-8 覆盖偏移 0..7
 9    LEN  PAYLOAD
 9+LEN 2   CRC16  CRC-16/CCITT-FALSE 覆盖偏移 0..8+LEN，小端
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from .crc import crc8, crc16

SOF0 = 0xA5
SOF1 = 0x5A
PROTO_MAJOR = 1
PROTO_MINOR = 0
PROTO_VERSION = (PROTO_MAJOR << 4) | PROTO_MINOR

HEADER_SIZE = 9          # 含 HCRC
TRAILER_SIZE = 2
OVERHEAD = HEADER_SIZE + TRAILER_SIZE
MAX_FRAME = 256          # 与固件 UART_DMA_BUFFER_SIZE 对齐
MAX_PAYLOAD = MAX_FRAME - OVERHEAD   # 245

_HDR = struct.Struct("<BBBBHBB")     # 不含 HCRC 的前 8 字节


@dataclass(frozen=True, slots=True)
class Frame:
    version: int
    msg_type: int
    seq: int
    flags: int
    payload: bytes

    @property
    def major(self) -> int:
        return self.version >> 4

    @property
    def minor(self) -> int:
        return self.version & 0x0F


def encode_frame(msg_type: int, seq: int, payload: bytes = b"", flags: int = 0,
                 version: int = PROTO_VERSION) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)} > {MAX_PAYLOAD}")
    head = _HDR.pack(SOF0, SOF1, version, msg_type & 0xFF, seq & 0xFFFF,
                     len(payload), flags & 0xFF)
    head += bytes((crc8(head),))
    body = head + bytes(payload)
    return body + struct.pack("<H", crc16(body))
