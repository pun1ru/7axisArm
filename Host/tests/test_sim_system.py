"""仿真后端与 README #6 第 6、8 条的系统级验证：
模式切换、仿真停止 / 发散、长时间运行的缓存上限、关闭时线程退出、切换后端时的安全动作。"""

import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from armhost.controller import MonitorController  # noqa: E402
from armhost.model.backend import MockBackend  # noqa: E402
from armhost.model.sim import HAVE_MUJOCO, KinematicEngine, SimBackend  # noqa: E402
from armhost.protocol import AckResult, StatusFlag  # noqa: E402


def wait_until(pred, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return False


def collect(b):
    got = []
    ev = []
    b.add_sample_listener(got.append)
    b.add_event_listener(lambda k, p: ev.append((k, p)))
    return got, ev


class TestSimKinematic(unittest.TestCase):
    def make(self, **kw):
        return SimBackend(prefer_mujoco=False, realtime=5.0, **kw)

    def test_hold_tracks_target(self):
        b = self.make()
        got, _ = collect(b)
        b.set_targets([30, -20, 10, 0, 0, 0, 0])
        b.start()
        try:
            self.assertTrue(wait_until(lambda: got and abs(got[-1].joints[0].angle_deg - 30) < 1.0, 10),
                            got[-1].joints[0].angle_deg if got else "no samples")
            self.assertAlmostEqual(got[-1].joints[1].angle_deg, -20, delta=1.5)
            ts = [s.mcu_time_us for s in got[-20:]]
            self.assertEqual(b.rate_hz, 200.0)
            self.assertTrue(all(np.diff(ts) == 5000), np.diff(ts))   # 200 Hz 仿真时间，严格均匀
        finally:
            b.stop()

    def test_free_mode_and_torque(self):
        b = self.make()
        got, _ = collect(b)
        b.set_mode("free")
        b.start()
        try:
            ack = b.set_joint_torque({0: 0.05, 2: 99.0}, hold_ms=400).result()
            self.assertEqual(ack.result, AckResult.OK_CLAMPED)
            self.assertEqual(ack.detail, 1 << 2)
            self.assertTrue(wait_until(lambda: got and got[-1].joints[0].angle_deg > 5, 5))
            self.assertTrue(StatusFlag.HOST_TORQUE_ACTIVE in StatusFlag(got[-1].status_flags))
            b.estop(True)
            self.assertEqual(b.set_joint_torque({0: 0.1}).result().result, AckResult.REJECT_ESTOP)
            self.assertTrue(wait_until(lambda: StatusFlag.ESTOP_LATCHED in StatusFlag(got[-1].status_flags)))
        finally:
            b.stop()

    def test_divergence_reported(self):
        e = KinematicEngine()
        b = SimBackend(engine=e, realtime=5.0)
        _, ev = collect(b)
        e.inertia[:] = 1e-12                                     # 故意让它发散
        b.set_targets([90] * 7)
        b.start()
        self.assertTrue(wait_until(lambda: any(k == "sim_error" for k, _ in ev), 3))
        self.assertTrue(wait_until(lambda: not b.running))
        b.stop()


@unittest.skipUnless(HAVE_MUJOCO, "没装 mujoco（pip install mujoco）")
class TestSimMujoco(unittest.TestCase):
    def test_model_loads_and_holds(self):
        b = SimBackend(realtime=5.0)
        self.assertEqual(b.engine.name, "MuJoCo")
        got, _ = collect(b)
        b.set_targets([20, 30, 0, -40, 0, 20, 0])
        b.start()
        try:
            self.assertTrue(wait_until(lambda: got and abs(got[-1].joints[1].angle_deg - 30) < 2, 8))
        finally:
            b.stop()


class TestSystem(unittest.TestCase):
    def test_mode_switch_cycle(self):
        """模拟设备 → 仿真 → 模拟设备 → 断开，反复切换：线程不泄漏，旧数据不串台"""
        c = MonitorController()
        base_threads = threading.active_count()
        try:
            for _ in range(3):
                c.set_backend(MockBackend())
                self.assertTrue(wait_until(lambda: c.buffer.size > 20))
                c.set_backend(SimBackend(prefer_mujoco=False))
                # 切换时清空：缓存里只能有仿真数据（仿真的 mcu_time 从 0 开始，seq 从 1 开始）
                self.assertTrue(wait_until(lambda: c.buffer.size > 20))
                s = c.latest()
                self.assertEqual(s.host_cmd_age_ms, 0xFFFF)
                self.assertLess(s.mcu_time_us, 60_000_000)
        finally:
            c.shutdown()
        self.assertTrue(wait_until(lambda: threading.active_count() <= base_threads + 1, 3),
                        [t.name for t in threading.enumerate()])

    def test_switch_away_sends_estop(self):
        """切走串口类后端时必须先发急停 + 清零（真机安全状态）"""
        c = MonitorController()
        mock = MockBackend()
        c.set_backend(mock)
        wait_until(lambda: mock.link.link_alive)
        dev = mock.device
        c.set_backend(None)
        names = [type(m).__name__ for m in dev.received]
        self.assertIn("Estop", names)
        self.assertIn("ClearTorque", names)
        self.assertTrue(dev.estop)

    def test_long_run_buffer_bounded(self):
        """缓存与记录队列有上限：500 Hz 跑一段时间，内存占用不随时间增长"""
        c = MonitorController(buffer_capacity=1000)
        c.set_backend(MockBackend(rate_hz=500))
        try:
            self.assertTrue(wait_until(lambda: c.buffer.total > 1500, 6))
            self.assertEqual(c.buffer.size, 1000)
            self.assertLessEqual(len(c._events), c._events.maxlen)
        finally:
            c.shutdown()


if __name__ == "__main__":
    unittest.main()
