"""LibSE/Module/Host 与上位机协议的一致性检查（不需要 C 编译器）。

做两件事：
1. 从 host.h / host.c 里解析所有偏移 / 尺寸宏，与 Python 的 struct 定义逐项比对；
2. 按 host.c 里的算法（逐字节小端写入、CRC 位算法）用 Python 镜像打一帧，与 Python 编码器输出比对。
改了 host.c 的偏移或算法，这里会报错，提醒同步两边。
"""

import re
import struct
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from armhost.protocol import (ArmState, JointFlag, JointRecord, JointState, MotorStatus, MsgType,  # noqa: E402
                              StatusFlag, encode_frame, FrameParser, decode_frame)
from armhost.protocol.crc import crc8, crc16  # noqa: E402
from armhost.protocol.msg_report import JointRecord as JR  # noqa: E402

HOST = ROOT.parent / "Real" / "DIYcontroller" / "LibSE" / "Module" / "Host"
H = (HOST / "host.h").read_text(encoding="utf-8")
C = (HOST / "host.c").read_text(encoding="utf-8")
SRC = H + "\n" + C


def macro(name: str) -> int:
    m = re.search(rf"#define\s+{name}\s+\(?\s*(0x[0-9A-Fa-f]+|\d+)U?\s*\)?", SRC)
    if not m:
        raise AssertionError(f"{name} 没找到")
    return int(m.group(1), 0)


def macro_expr(name: str, env: dict) -> int:
    m = re.search(rf"#define\s+{name}\s+\((.*?)\)\s*(?:/\*.*)?$", SRC, re.M)
    expr = re.sub(r"(\d+)U", r"\1", m.group(1))
    return int(eval(expr, {}, env))


class TestHostModuleSync(unittest.TestCase):
    def test_frame_layout_matches_protocol(self):
        self.assertEqual(macro("HOST_FRAME_SOF0"), 0xA5)
        self.assertEqual(macro("HOST_FRAME_SOF1"), 0x5A)
        self.assertEqual(macro("HOST_FRAME_VERSION"), 0x10)
        self.assertEqual(macro("HOST_FRAME_HEADER_SIZE"), 9)
        self.assertEqual(macro("HOST_FRAME_TRAILER_SIZE"), 2)
        self.assertEqual(macro("HOST_MSG_JOINT_STATE"), int(MsgType.JOINT_STATE))
        # 帧头字段偏移：SOF0 SOF1 VER TYPE SEQ(2) LEN FLAGS HCRC
        offs = {n: macro(f"HOST_OFFSET_{n}") for n in
                ("SOF0", "SOF1", "VERSION", "TYPE", "SEQUENCE", "LENGTH", "FLAGS", "HEADER_CRC")}
        self.assertEqual(offs, dict(SOF0=0, SOF1=1, VERSION=2, TYPE=3, SEQUENCE=4, LENGTH=6, FLAGS=7, HEADER_CRC=8))
        self.assertEqual(macro("HOST_HEADER_CRC_SPAN"), 8)

    def test_joint_state_offsets_match_python_structs(self):
        self.assertEqual(macro("HOST_JOINT_RECORD_SIZE"), JR.S.size)
        self.assertEqual(macro("HOST_JOINT_STATE_HEAD_SIZE"), JointState.H.size)
        # 负载头：<IBBBBHH  mcu_time(0) state(4) count(5) rec_size(6) rsv(7) flags(8) age(10)
        self.assertEqual([macro(f"HOST_HEAD_OFFSET_{n}") for n in
                          ("TIME", "ARM_STATE", "COUNT", "REC_SIZE", "RESERVED", "FLAGS", "CMD_AGE")],
                         [0, 4, 5, 6, 7, 8, 10])
        # 记录 <fffffHBBbBH：5 个 f32 = 20 字节，之后 H B B b B H
        self.assertEqual([macro(f"HOST_REC_OFFSET_{n}") for n in
                          ("ANGLE", "VELOCITY", "TORQUE", "TARGET", "TORQUE_CMD", "ENCODER", "STATUS",
                           "ERROR_ID", "TEMP", "FLAGS", "FEEDBACK")],
                         [0, 4, 8, 12, 16, 20, 22, 23, 24, 25, 26])
        self.assertEqual(macro("HOST_CMD_AGE_NONE"), 0xFFFF)

    def test_flags_and_fixed_size(self):
        self.assertEqual(macro("HOST_FLAG_CTRL_ALIGNING"), int(StatusFlag.CTRL_ALIGN_PENDING))
        self.assertEqual(macro("HOST_JOINT_FLAG_ONLINE"), int(JointFlag.ONLINE))
        self.assertEqual(7 * macro("HOST_JOINT_RECORD_SIZE") + macro("HOST_JOINT_STATE_HEAD_SIZE"), 208)
        self.assertLessEqual(219, 256)
        self.assertIn("ARM_JOINT_COUNT", H)

    def test_crc_constants(self):
        self.assertEqual((macro("HOST_CRC8_POLY"), macro("HOST_CRC8_INIT")), (0x07, 0x00))
        self.assertEqual((macro("HOST_CRC16_POLY"), macro("HOST_CRC16_INIT")), (0x1021, 0xFFFF))

    # ------------------------------------------------------------ 镜像 host.c 的算法
    @staticmethod
    def mirror_crc8(data):
        c = 0
        for b in data:
            c ^= b
            for _ in range(8):
                c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        return c

    @staticmethod
    def mirror_crc16(data):
        c = 0xFFFF
        for b in data:
            c ^= b << 8
            for _ in range(8):
                c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        return c

    def test_crc_mirror_agrees_with_protocol(self):
        self.assertEqual(self.mirror_crc8(b"123456789"), 0xF4)
        self.assertEqual(self.mirror_crc16(b"123456789"), 0x29B1)
        blob = bytes(range(256)) * 3
        self.assertEqual(self.mirror_crc8(blob), crc8(blob))
        self.assertEqual(self.mirror_crc16(blob), crc16(blob))

    def test_mirror_frame_equals_python_encoder_and_decodes(self):
        """按 host.c 的写法逐字节拼帧，必须与 Python 编码器逐字节相等，且能被上位机解析器解出同样的值"""
        recs = [JointRecord(angle_deg=12.5 + i, velocity_dps=-3.0 * i, torque_nm=0.25 * i, target_deg=10.0 + i,
                            torque_cmd_nm=0.1 * i, encoder=0x1200 + i, motor_status=MotorStatus.ENABLED,
                            error_id=i % 3, temperature_c=30 + i, joint_flags=JointFlag.ONLINE,
                            feedback_count=1000 + i) for i in range(7)]
        js = JointState(123456, ArmState.ENABLE, StatusFlag.CTRL_ALIGN_PENDING, 0xFFFF, recs)
        expect = encode_frame(MsgType.JOINT_STATE, 0xFFFE, js.encode())

        def f32(x):
            return struct.pack("<f", x)

        payload = bytearray(208)
        payload[0:4] = struct.pack("<I", 123456)
        payload[4], payload[5], payload[6], payload[7] = int(ArmState.ENABLE), 7, 28, 0
        payload[8:10] = struct.pack("<H", int(StatusFlag.CTRL_ALIGN_PENDING))
        payload[10:12] = struct.pack("<H", 0xFFFF)
        for i, r in enumerate(recs):
            o = 12 + 28 * i
            payload[o + 0:o + 4] = f32(r.angle_deg)
            payload[o + 4:o + 8] = f32(r.velocity_dps)
            payload[o + 8:o + 12] = f32(r.torque_nm)
            payload[o + 12:o + 16] = f32(r.target_deg)
            payload[o + 16:o + 20] = f32(r.torque_cmd_nm)
            payload[o + 20:o + 22] = struct.pack("<H", r.encoder)
            payload[o + 22], payload[o + 23] = int(r.motor_status), r.error_id
            payload[o + 24] = r.temperature_c & 0xFF
            payload[o + 25] = int(r.joint_flags)
            payload[o + 26:o + 28] = struct.pack("<H", r.feedback_count)
        frame = bytearray(9 + 208 + 2)
        frame[0:8] = bytes([0xA5, 0x5A, 0x10, 0x10, 0xFE, 0xFF, 208, 0])
        frame[8] = self.mirror_crc8(frame[:8])
        frame[9:9 + 208] = payload
        frame[9 + 208:] = struct.pack("<H", self.mirror_crc16(frame[:9 + 208]))
        self.assertEqual(bytes(frame), expect)

        (fr,) = FrameParser().feed(bytes(frame))
        got = decode_frame(fr)
        # 线上是 float32，期望值先过一遍 float32 再比
        for g, r in zip(got.joints, recs):
            for k in ("angle_deg", "velocity_dps", "torque_nm", "target_deg", "torque_cmd_nm"):
                self.assertEqual(getattr(g, k), struct.unpack("<f", struct.pack("<f", getattr(r, k)))[0], k)
            for k in ("encoder", "motor_status", "error_id", "temperature_c", "joint_flags", "feedback_count"):
                self.assertEqual(getattr(g, k), getattr(r, k), k)
        self.assertEqual(got.arm_state, ArmState.ENABLE)

    def test_source_follows_conventions(self):
        """LibSE 代码规范里能机器检查的几条"""
        self.assertNotIn("/**", SRC, "不使用 doxygen 注释")
        self.assertNotRegex(C, r"\)\s*\n\s*\{\s*\n" if False else r"\bxTaskCreate|xQueueCreate|xSemaphoreCreate",
                            "Module 层不创建任务 / 队列 / 信号量")
        self.assertNotIn("#include \"system_", SRC, "Module 不引用 System 层")
        # allman：函数 / 控制语句的左花括号单独一行（排除 typedef struct { 与初始化）
        bad = [ln for ln in C.splitlines()
               if re.match(r"\s*(if|for|while|else)\b.*\{\s*$", ln) or re.match(r"^\w[\w\s\*]*\)\s*\{\s*$", ln)]
        self.assertEqual(bad, [])


if __name__ == "__main__":
    unittest.main()
