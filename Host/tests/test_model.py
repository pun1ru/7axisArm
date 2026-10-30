"""状态模型、CSV 记录/回放、控制器测试（不需要 Qt）。"""

import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from armhost.controller import MonitorController  # noqa: E402
from armhost.model import SampleBuffer, channel_stats, timing_stats  # noqa: E402
from armhost.model.backend import MockBackend, ReplayBackend  # noqa: E402
from armhost.model.recorder import CsvRecorder, load_csv  # noqa: E402
from armhost.model.state import RobotSample  # noqa: E402
from armhost.protocol import JointRecord  # noqa: E402


def mk(i, t_us=None, host_ns=None, n=7):
    t_us = i * 5000 if t_us is None else t_us
    host_ns = 1_000_000_000 + i * 5_000_000 if host_ns is None else host_ns
    return RobotSample(t_us, host_ns, i & 0xFFFF, 1, 3, 0xFFFF,
                       tuple(JointRecord(angle_deg=float(i + j), velocity_dps=float(j), torque_nm=0.1 * j,
                                         target_deg=float(i), feedback_count=i) for j in range(n)))


def wait_until(pred, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return False


class TestBuffer(unittest.TestCase):
    def test_wrap_and_order(self):
        b = SampleBuffer(capacity=100)
        for i in range(250):
            b.append(mk(i))
        v = b.view()
        self.assertEqual(len(v), 100)
        self.assertTrue(np.all(np.diff(v.mcu_s) > 0))
        self.assertAlmostEqual(v.channel(0, "angle_deg")[-1], 249.0)
        self.assertAlmostEqual(v.channel(3, "angle_deg")[0], 153.0)

    def test_window(self):
        b = SampleBuffer(capacity=1000)
        for i in range(1000):          # 5 s @ 200 Hz
            b.append(mk(i))
        v = b.view(last_s=1.0)
        self.assertAlmostEqual(v.mcu_s[-1] - v.mcu_s[0], 1.0, places=2)

    def test_mcu_vs_host_time_are_separate(self):
        b = SampleBuffer(capacity=100)
        # MCU 均匀 5 ms 采样，上位机成批到达（每 4 帧一起）
        for i in range(40):
            b.append(mk(i, host_ns=1_000_000_000 + (i // 4) * 20_000_000))
        t = timing_stats(b.view())
        self.assertAlmostEqual(t.mcu_rate_hz, 200.0, delta=1)
        self.assertAlmostEqual(t.mcu_dt_max_ms, 5.0, delta=0.01)
        self.assertAlmostEqual(t.host_gap_max_ms, 20.0, delta=0.01)
        self.assertGreater(t.jitter_ms, 1.0)

    def test_restart_clears(self):
        b = SampleBuffer(capacity=100)
        for i in range(10):
            b.append(mk(i, t_us=10_000_000 + i * 5000))
        b.append(mk(0, t_us=1000))
        self.assertEqual(b.size, 1)
        self.assertEqual(b.restarts, 1)

    def test_joint_count_change(self):
        b = SampleBuffer(capacity=10)
        b.append(mk(0))
        b.append(mk(1, n=3))
        self.assertEqual(b.joint_count, 3)
        self.assertEqual(b.size, 1)

    def test_channel_stats(self):
        b = SampleBuffer(capacity=100)
        for i in range(10):
            b.append(mk(i))
        (s,) = channel_stats(b.view(), [(0, "angle_deg")])
        self.assertEqual((s.n, s.min, s.max, s.mean), (10, 0.0, 9.0, 4.5))


class TestRecorder(unittest.TestCase):
    def test_roundtrip_and_replay(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "a.csv"
            r = CsvRecorder()
            r.start(path)
            for i in range(200):
                r.put(mk(i))
            r.stop()
            self.assertEqual(r.rows, 200)
            back = load_csv(path)
            self.assertEqual(len(back), 200)
            self.assertEqual(back[7].mcu_time_us, 35000)
            self.assertEqual(back[7].host_time_ns, mk(7).host_time_ns)
            self.assertAlmostEqual(back[7].joints[3].angle_deg, 10.0)

            got = []
            rb = ReplayBackend(path, speed=10.0)
            rb.add_sample_listener(got.append)
            fin = []
            rb.add_event_listener(lambda k, p: fin.append(k))
            rb.start()
            self.assertTrue(wait_until(lambda: "replay_finished" in fin))
            rb.stop()
            self.assertEqual(len(got), 200)
            self.assertEqual([s.mcu_time_us for s in got[:3]], [0, 5000, 10000])

    def test_bad_csv(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.csv"
            p.write_text("a,b\n1,2\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                ReplayBackend(p)


class TestController(unittest.TestCase):
    def test_mock_end_to_end(self):
        c = MonitorController()
        c.set_backend(MockBackend(rate_hz=200))
        try:
            self.assertTrue(wait_until(lambda: c.buffer.size > 100))
            self.assertIsNotNone(c.latest())
            self.assertTrue(wait_until(lambda: any(k == "device_info" for k, *_ in list(c._events))))
            st = c.status()
            self.assertTrue(st["link_alive"])
            c.estop(True)
            self.assertTrue(wait_until(lambda: any(k == "estop" for k, *_ in list(c._events))))
        finally:
            c.shutdown()
        self.assertIsNone(c.backend)

    def test_switch_backend_ignores_stale_callbacks(self):
        c = MonitorController()
        old = MockBackend()
        c.set_backend(old)
        wait_until(lambda: c.buffer.size > 10)
        c.set_backend(None)
        old._emit_sample(mk(1))          # 旧后端残留回调
        self.assertEqual(c.buffer.size, 0)


if __name__ == "__main__":
    unittest.main()


class TestConfig(unittest.TestCase):
    def test_defaults_and_errors(self):
        from armhost import config
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "s.toml"
            self.assertEqual(config.load(p), config.DEFAULTS)            # 没有文件
            p.write_text("[serial]\nport = \"COM9\"\n[sim]\nrealtime = 2\n", encoding="utf-8")
            c = config.load(p)
            self.assertEqual(c["serial"]["port"], "COM9")
            self.assertEqual(c["serial"]["baudrate"], 921600)
            self.assertEqual(c["sim"]["realtime"], 2.0)
            for bad in ("[serial\n", "[nope]\na=1\n", "[serial]\nbaud=1\n", "[serial]\nbaudrate=\"x\"\n"):
                p.write_text(bad, encoding="utf-8")
                with self.assertRaises(config.ConfigError):
                    config.load(p)

    def test_shipped_settings_valid(self):
        from armhost import config
        config.load()          # 仓库里那份 settings.toml 必须能读
