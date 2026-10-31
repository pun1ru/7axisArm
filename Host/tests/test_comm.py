"""通信层测试：序号、时钟、传输、链路会话（基于 SimDevice 模拟设备）。"""

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from armhost.comm import (AckTimeoutError, ArmLink, CommandRejected, FaultConfig,  # noqa: E402
                          LinkLostError, LinkState, McuClock, SeqTracker, SimDevice,
                          make_mock_transport)
from armhost.protocol import AckResult, ArmState, JointFlag, MsgType, StatusFlag  # noqa: E402


def wait_until(pred, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.005)
    return False


class TestSeq(unittest.TestCase):
    def test_wrap(self):
        t = SeqTracker()
        for s in (65534, 65535, 0, 1):
            self.assertEqual(t.update(s), 0)
        self.assertEqual(t.lost, 0)

    def test_loss_across_wrap(self):
        t = SeqTracker()
        t.update(65534)
        self.assertEqual(t.update(2), 3)       # 65535, 0, 1 丢失

    def test_duplicate_and_restart(self):
        t = SeqTracker()
        t.update(100)
        t.update(100)
        t.update(5)                            # 后退大于半圈 -> 视为对端重启
        self.assertEqual((t.duplicate, t.resync, t.lost), (1, 1, 0))

    def test_mcu_clock_unwrap(self):
        c = McuClock()
        self.assertEqual(c.unwrap(0xFFFFFF00), 0xFFFFFF00)
        self.assertEqual(c.unwrap(0x10), (1 << 32) + 0x10)


class LinkCase(unittest.TestCase):
    faults = None
    rate = 200

    def setUp(self):
        self.dev = SimDevice(state_rate_hz=self.rate, faults=self.faults or FaultConfig())
        self.tr, _, self.fac = make_mock_transport(self.dev)
        self.link = ArmLink(self.tr, ack_timeout_s=0.1)
        self.dev.start()
        self.link.start()
        self.assertTrue(self.tr.wait_connected(1.0))
        self.assertTrue(wait_until(lambda: self.link.link_alive))

    def tearDown(self):
        self.link.stop()
        self.dev.stop()


class TestLink(LinkCase):
    def test_device_info(self):
        info = self.link.request_device_info().result(1.0)
        self.assertEqual(info.joint_count, 7)
        self.assertEqual(info.joint_record_size, 28)
        self.assertEqual(self.link.device_info, info)

    def test_joint_state_stream_and_mirror(self):
        got = []
        self.link.subscribe(MsgType.JOINT_STATE, got.append)
        time.sleep(0.3)
        self.assertGreater(len(got), 30)
        snap = self.link.mirror.snapshot()
        self.assertEqual(len(snap.arm.feedback), 7)
        self.assertEqual(snap.arm.state, ArmState.ENABLE)
        self.assertIn("observer.joint_deg", self.link.mirror.format_c())
        st = self.link.snapshot_stats()
        self.assertEqual(st["lost_frames"], 0)
        self.assertEqual(st["parser"]["crc_errors"], 0)

    def test_torque_ack_and_clamp(self):
        ack = self.link.set_joint_torque({0: 0.5, 6: 100.0}).result(1.0)
        self.assertEqual(ack.result, AckResult.OK_CLAMPED)
        self.assertEqual(ack.detail, 1 << 6)
        self.assertTrue(wait_until(
            lambda: StatusFlag.HOST_TORQUE_ACTIVE in self.link.mirror.snapshot().arm.status_flags))

    def test_torque_expires(self):
        self.link.set_joint_torque({1: 0.5}, hold_ms=50).result(1.0)
        self.assertTrue(wait_until(
            lambda: StatusFlag.HOST_TORQUE_ACTIVE not in self.link.mirror.snapshot().arm.status_flags, 1.0))

    def test_estop_rejects_torque(self):
        self.link.estop(True).result(1.0)
        with self.assertRaises(CommandRejected) as cm:
            self.link.set_joint_torque({0: 0.1}).result(1.0)
        self.assertEqual(cm.exception.result, AckResult.REJECT_ESTOP)
        self.link.estop(False).result(1.0)
        self.link.set_joint_torque({0: 0.1}).result(1.0)

    def test_param_rw(self):
        self.link.write_param(0x0002, 0, 20).result(1.0)             # 唯一可写的是上报频率
        self.assertEqual(self.link.read_param(0x0002, 0).result(1.0).value, 20)
        with self.assertRaises(CommandRejected):
            self.link.write_param(0x0100, 0, 1).result(1.0)       # 只读
        with self.assertRaises(CommandRejected):
            self.link.read_param(0x0100, 99).result(1.0)          # 越界
        dh = self.link.read_param(0x0400, 2).result(1.0)          # J0 d_m
        self.assertAlmostEqual(dh.value, 0.0626, places=5)

    def test_gain_limit_params_read_only(self):
        """PID 增益、关节限位、上位机力矩限幅：上位机只能读，写一律 READ_ONLY 且值不变"""
        for pid in (0x0102, 0x0103, 0x0200, 0x0201, 0x0202, 0x0203, 0x0204, 0x0205, 0x0300):
            with self.subTest(hex(pid)):
                before = self.link.read_param(pid, 2).result(1.0)
                with self.assertRaises(CommandRejected) as cm:
                    self.link.write_param(pid, 2, 123.0).result(1.0)
                self.assertEqual(cm.exception.result, AckResult.READ_ONLY)
                self.assertEqual(self.link.read_param(pid, 2).result(1.0).value, before.value)

    def test_host_target_updates_one_joint_and_loop_mask(self):
        self.dev.limit[0x0102] = [-90.0, -95.0, -180.0, -120.0, -180.0, -60.0, -180.0]
        self.dev.limit[0x0103] = [90.0, 100.0, 180.0, 0.0, 180.0, 90.0, 180.0]
        with self.assertRaises(CommandRejected):
            self.link.set_host_target(0, 10.0, True).result(1.0)   # still ENABLE
        self.dev.arm_state = ArmState.HOST
        ack = self.link.set_host_target(0, 10.0, True).result(1.0)
        self.assertEqual(ack.result, AckResult.OK)
        self.assertEqual(self.dev.host_closed_loop_mask, 1 << 0)
        self.assertEqual(self.dev.joints[0].host_target, 10.0)

        ack = self.link.set_host_target(1, 20.0, False).result(1.0)
        self.assertEqual(ack.result, AckResult.OK)
        self.assertEqual(self.dev.host_closed_loop_mask, 1 << 0)  # J1 只存目标，不改变 J0
        self.assertEqual((self.dev.joints[0].host_target, self.dev.joints[1].host_target), (10.0, 20.0))
        self.assertTrue(wait_until(
            lambda: StatusFlag.HOST_CLOSED_LOOP in self.link.mirror.snapshot().arm.status_flags))
        with self.assertRaises(CommandRejected) as cm:
            self.link.set_host_target(0, 91.0, True).result(1.0)
        self.assertEqual(cm.exception.result, AckResult.BAD_PARAM)
        self.assertEqual(self.dev.joints[0].host_target, 10.0)

        self.assertEqual(self.link.stop_host_loops().result(1.0).result, AckResult.OK)
        self.assertEqual(self.dev.host_closed_loop_mask, 0)
        self.assertEqual(self.dev.joints[1].host_target, 20.0)
        self.assertTrue(wait_until(
            lambda: not (self.link.mirror.snapshot().arm.joint_flags[1]
                         & JointFlag.HOST_CLOSED_LOOP)))

    def test_ack_timeout_and_retry(self):
        self.dev.ignore_types = {MsgType.PARAM_READ}
        with self.assertRaises(AckTimeoutError):
            self.link.read_param(0x0001).result(2.0)
        n = sum(1 for m in self.dev.received if type(m).__name__ == "ParamRead")
        self.assertEqual(n, 3)                                    # 1 次 + 2 次重试

    def test_torque_never_retried(self):
        self.dev.ignore_types = {MsgType.SET_JOINT_TORQUE}
        with self.assertRaises(AckTimeoutError):
            self.link.set_joint_torque({0: 0.1}).result(2.0)
        n = sum(1 for m in self.dev.received if type(m).__name__ == "SetJointTorque")
        self.assertEqual(n, 1)

    def test_device_link_timeout_clears_torque(self):
        self.link.set_joint_torque({0: 0.5}, hold_ms=500).result(1.0)
        self.assertTrue(wait_until(lambda: self.dev._link_ok))   # 先确认设备侧认为链路正常
        self.link._stop.set()                 # 停心跳，模拟上位机卡死
        self.assertTrue(wait_until(lambda: not self.dev._link_ok, 1.5))
        self.assertEqual(self.dev.joints[0].host_tau, 0.0)

    def test_unplug_and_reconnect(self):
        states = []
        self.tr.add_state_listener(lambda s, r: states.append(s))
        fut = self.link.read_param(0x0001)
        self.dev.ignore_types = {MsgType.PARAM_READ}
        self.fac.unplug()
        self.assertTrue(wait_until(lambda: self.tr.state == LinkState.RECONNECTING))
        self.assertFalse(self.tr.write(b"x"))                     # 断线时不排队
        self.fac.plug()
        self.dev.ignore_types = set()
        self.assertTrue(self.tr.wait_connected(2.0))
        self.assertTrue(wait_until(lambda: self.link.link_alive))
        self.assertTrue(fut.done())
        if fut.exception() is not None:
            self.assertIsInstance(fut.exception(), (LinkLostError, AckTimeoutError))
        self.assertIn(LinkState.RECONNECTING, states)
        self.link.request_device_info().result(1.0)

    def test_not_connected_fails_fast(self):
        self.fac.unplug()
        wait_until(lambda: not self.tr.is_connected)
        with self.assertRaises(LinkLostError):
            self.link.estop().result(0.5)
        self.fac.plug()


class TestLinkWithFaults(LinkCase):
    faults = FaultConfig(noise_prob=0.05, corrupt_prob=0.03, drop_prob=0.03, chunk_max=17, seed=1)

    def test_survives_noise(self):
        time.sleep(1.0)
        st = self.link.snapshot_stats()
        p = st["parser"]
        self.assertGreater(p["frames_ok"], 120)
        self.assertGreater(p["crc_errors"] + p["header_errors"] + p["bytes_discarded"], 0)
        self.assertGreater(st["lost_frames"], 0)
        # 损坏的流里命令仍能完成（靠重试）
        self.link.read_param(0x0001).result(2.0)


class TestBurst(LinkCase):
    rate = 500

    def test_500hz(self):
        got = []
        self.link.subscribe(MsgType.JOINT_STATE, got.append)
        time.sleep(1.0)
        self.assertGreater(len(got), 350)
        self.assertEqual(self.link.snapshot_stats()["parser"]["crc_errors"], 0)


if __name__ == "__main__":
    unittest.main()
