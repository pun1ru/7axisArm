"""协议层单元测试：CRC、帧、解析器、消息编解码。纯 unittest，pytest 也能直接收集。"""

import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from armhost.protocol import (MAX_PAYLOAD, Ack, AckResult, ArmState, CanBusStatus, ClearTorque,  # noqa: E402
                              DecodeError, DeviceInfo, Estop, EstopAction, Event, EventCode,
                              FrameParser, Heartbeat, JointFlag, JointRecord, JointState,
                              MotorStatus, MsgType, ParamRead, ParamValue, ParamWrite, RawMessage,
                              SetJointTorque, StatusFlag, SystemStatus, TorqueItem, decode_frame,
                              decode_payload, encode_frame)
from armhost.protocol.crc import crc8, crc16  # noqa: E402
from armhost.protocol.enums import ParamType  # noqa: E402


def sample_joint_state(n=7):
    return JointState(123456, ArmState.ENABLE, StatusFlag.RC_ONLINE | StatusFlag.HOST_LINK_OK, 15,
                      [JointRecord(i * 10.0, 1.5, -0.25, i * 10.0, 0.5, 1000 + i, MotorStatus.ENABLED,
                                   0, 30 + i, JointFlag.ONLINE, i) for i in range(n)])


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class TestCrc(unittest.TestCase):
    def test_check_values(self):
        self.assertEqual(crc8(b"123456789"), 0xF4)
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_empty(self):
        self.assertEqual(crc8(b""), 0x00)
        self.assertEqual(crc16(b""), 0xFFFF)


class TestFrame(unittest.TestCase):
    def test_layout(self):
        f = encode_frame(MsgType.HEARTBEAT, 0x1234, b"\x01\x02", flags=1)
        self.assertEqual(f[:2], b"\xA5\x5A")
        self.assertEqual(f[2], 0x10)
        self.assertEqual(f[3], 0x01)
        self.assertEqual(f[4:6], b"\x34\x12")
        self.assertEqual(f[6], 2)
        self.assertEqual(f[7], 1)
        self.assertEqual(f[8], crc8(f[:8]))
        self.assertEqual(struct.unpack("<H", f[-2:])[0], crc16(f[:-2]))
        self.assertEqual(len(f), 9 + 2 + 2)

    def test_max_payload(self):
        self.assertEqual(MAX_PAYLOAD, 245)
        encode_frame(1, 0, bytes(MAX_PAYLOAD))
        with self.assertRaises(ValueError):
            encode_frame(1, 0, bytes(MAX_PAYLOAD + 1))

    def test_joint_state_fits(self):
        f = encode_frame(MsgType.JOINT_STATE, 0, sample_joint_state().encode())
        self.assertEqual(len(f), 9 + 12 + 7 * 28 + 2)
        self.assertLessEqual(len(f), 256)


class TestParser(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.p = FrameParser(clock=self.clock)
        self.f1 = encode_frame(MsgType.HEARTBEAT, 1, Heartbeat(10, 20).encode())
        self.f2 = encode_frame(MsgType.JOINT_STATE, 2, sample_joint_state().encode())

    def test_byte_by_byte(self):
        out = []
        for b in self.f1 + self.f2:
            out += self.p.feed(bytes([b]))
        self.assertEqual([f.seq for f in out], [1, 2])

    def test_multiple_in_one_chunk(self):
        out = self.p.feed(self.f1 + self.f2 + self.f1)
        self.assertEqual(len(out), 3)

    def test_garbage_prefix_and_between(self):
        out = self.p.feed(b"\x00\xA5\x11\x5A\xA5" + self.f1 + b"\xA5\xA5\x5A" + self.f2)
        self.assertEqual([f.seq for f in out], [1, 2])
        self.assertGreater(self.p.stats.bytes_discarded, 0)

    def test_crc_error_resync(self):
        bad = bytearray(self.f2)
        bad[20] ^= 0x40
        out = self.p.feed(bytes(bad) + self.f1)
        self.assertEqual([f.seq for f in out], [1])
        self.assertEqual(self.p.stats.crc_errors, 1)

    def test_header_error(self):
        bad = bytearray(self.f1)
        bad[6] = 200        # 破坏 LEN，HCRC 必须拦住，不能空等 200 字节
        out = self.p.feed(bytes(bad) + self.f2)
        self.assertEqual([f.seq for f in out], [2])
        self.assertGreaterEqual(self.p.stats.header_errors, 1)

    def test_frame_hidden_inside_bad_frame(self):
        # 截断的半帧（帧头完好）后面紧跟好帧：解析器先按 LEN 等字节，凑满后整帧 CRC 失败，
        # 丢 1 字节重新同步，必须能把夹在里面的好帧找回来，而不是整段跳过
        out = self.p.feed(self.f2[:40] + self.f1 + self.f1 + self.f2 + self.f2)
        seqs = [f.seq for f in out]
        self.assertEqual(seqs.count(1), 2)
        self.assertEqual(seqs.count(2), 2)
        self.assertEqual(self.p.stats.crc_errors, 1)

    def test_truncated_frame_waits_then_times_out(self):
        # 只有半帧且后续字节不再到来：靠 stale_timeout 丢弃，后面的帧不受影响
        self.assertEqual(self.p.feed(self.f2[:40] + self.f1), [])
        self.clock.t = 1.0
        self.assertEqual([f.seq for f in self.p.feed(self.f1)], [1])

    def test_length_error(self):
        h = bytearray(b"\xA5\x5A\x10\x01\x00\x00\xF6\x00")
        h.append(crc8(h))
        out = self.p.feed(bytes(h) + self.f1)
        self.assertEqual(self.p.stats.length_errors, 1)
        self.assertEqual(len(out), 1)

    def test_version_mismatch(self):
        f = encode_frame(MsgType.HEARTBEAT, 9, Heartbeat().encode(), version=0x20)
        out = self.p.feed(f + self.f1)
        self.assertEqual([x.seq for x in out], [1])
        self.assertEqual(self.p.stats.version_errors, 1)

    def test_minor_version_accepted(self):
        f = encode_frame(MsgType.HEARTBEAT, 9, Heartbeat().encode(), version=0x13)
        out = self.p.feed(f)
        self.assertEqual(out[0].minor, 3)

    def test_stale_partial_dropped(self):
        self.p.feed(self.f2[:50])
        self.clock.t = 1.0
        out = self.p.feed(self.f1)
        self.assertEqual(len(out), 1)
        self.assertEqual(self.p.stats.timeout_drops, 1)

    def test_split_sof(self):
        out = self.p.feed(self.f1[:1])
        out += self.p.feed(self.f1[1:])
        self.assertEqual(len(out), 1)


class TestMessages(unittest.TestCase):
    def roundtrip(self, m):
        f = encode_frame(m.TYPE, 7, m.encode())
        (fr,) = FrameParser().feed(f)
        self.assertEqual(decode_frame(fr), m)

    def test_roundtrip_all(self):
        for m in [Heartbeat(1, 2), DeviceInfo(fw_name="v610"), Ack(5, 0x20, AckResult.OK_CLAMPED, 3),
                  sample_joint_state(), sample_joint_state(1),
                  SystemStatus(1, 2, ArmState.ENABLE, 0, 3, 3, StatusFlag.RC_ONLINE, 1, 2, 3, 4,
                               [0, 1, 2], [CanBusStatus(1, 2, 3, 4, 5)] * 3),
                  Event(1, EventCode.MOTOR_FAULT, 2, 3, -4),
                  SetJointTorque(100, [TorqueItem(0, 1.5), TorqueItem(6, -0.5)]),
                  ClearTorque(0x7F), Estop(EstopAction.LATCH), Estop(EstopAction.RELEASE),
                  ParamRead(0x200, 3), ParamWrite(0x200, 3, ParamType.F32, 1.5),
                  ParamWrite(0x0001, 0, ParamType.U16, 200),
                  ParamValue(9, 0x0101, 2, ParamType.I8, AckResult.OK, 1, -1)]:
            with self.subTest(type(m).__name__):
                self.roundtrip(m)

    def test_short_payload_raises(self):
        with self.assertRaises(DecodeError):
            JointState.decode(sample_joint_state().encode()[:-1])
        with self.assertRaises(DecodeError):
            Ack.decode(b"\x00" * 7)

    def test_forward_compat_longer_record(self):
        # 模拟 v1.1：每条记录尾部多 4 字节、负载尾部多 2 字节
        js = sample_joint_state(2)
        head = bytearray(js.encode()[:12])
        head[6] = 32
        body = b"".join(r.encode() + b"\xEE" * 4 for r in js.joints)
        got = JointState.decode(bytes(head) + body + b"\x00\x00")
        self.assertEqual(got.joints, js.joints)

    def test_unknown_type_and_enum(self):
        self.assertIsInstance(decode_payload(0x7E, b"\x01"), RawMessage)
        a = Ack.decode(Ack(1, 2, 0, 0).encode()[:3] + b"\x63" + b"\x00" * 4)
        self.assertEqual(a.result, 0x63)

    def test_joint_state_offsets(self):
        p = sample_joint_state().encode()
        self.assertEqual(struct.unpack_from("<I", p, 0)[0], 123456)
        self.assertEqual(p[4], ArmState.ENABLE)
        self.assertEqual(p[5], 7)
        self.assertEqual(p[6], 28)
        # 第 2 个关节 (J1) 的 angle_deg 在 12 + 28
        self.assertAlmostEqual(struct.unpack_from("<f", p, 12 + 28)[0], 10.0)
        # J1 encoder 在 12 + 28 + 20
        self.assertEqual(struct.unpack_from("<H", p, 12 + 28 + 20)[0], 1001)


if __name__ == "__main__":
    unittest.main()
