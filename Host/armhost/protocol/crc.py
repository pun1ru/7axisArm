"""CRC 算法，与 firmware_ref/arm_proto.c 逐位一致。

* 帧头：CRC-8/SMBUS   poly=0x07 init=0x00 refin=F refout=F xorout=0x00  check("123456789")=0xF4
* 整帧：CRC-16/CCITT-FALSE poly=0x1021 init=0xFFFF refin=F refout=F xorout=0x0000 check=0x29B1
"""

from __future__ import annotations


def _make_crc8_table() -> tuple[int, ...]:
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return tuple(table)


def _make_crc16_table() -> tuple[int, ...]:
    table = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        table.append(c)
    return tuple(table)


_CRC8 = _make_crc8_table()
_CRC16 = _make_crc16_table()


def crc8(data: bytes | bytearray | memoryview, init: int = 0x00) -> int:
    c = init
    for b in data:
        c = _CRC8[c ^ b]
    return c


def crc16(data: bytes | bytearray | memoryview, init: int = 0xFFFF) -> int:
    c = init
    for b in data:
        c = ((c << 8) & 0xFFFF) ^ _CRC16[((c >> 8) ^ b) & 0xFF]
    return c
