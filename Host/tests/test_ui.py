"""界面冒烟测试：offscreen 平台下构造主窗口，接模拟设备跑一段时间。

    python -m unittest tests.test_ui
截图会存到 logs/ui_smoke.png，方便目视检查布局。
"""

import os
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from armhost.model.backend import MockBackend  # noqa: E402
from armhost.ui import theme  # noqa: E402
from armhost.ui.main_window import MonitorWindow  # noqa: E402

app = QApplication.instance() or QApplication([])
theme.apply(app)


def pump(seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        QCoreApplication.processEvents()
        time.sleep(0.005)


class TestDecimate(unittest.TestCase):
    def test_keeps_peaks_and_latest(self):
        import numpy as np
        from armhost.ui.scope import decimate_minmax
        x = np.arange(10007, dtype=float)
        y = np.zeros((10007, 2), np.float32)
        y[1234, 0] = 99.0          # 单点尖峰
        y[5000, 1] = -42.0
        y[-1, 0] = 7.0             # 最新样本
        x2, y2 = decimate_minmax(x, y, 500)
        self.assertEqual(len(x2), 1000)
        self.assertEqual(y2[:, 0].max(), 99.0)
        self.assertEqual(y2[:, 1].min(), -42.0)
        self.assertEqual(x2[-1], x[-1])
        self.assertEqual(y2[-1, 0], 7.0)
        self.assertTrue(np.all(np.diff(x2) >= 0))

    def test_short_passthrough(self):
        import numpy as np
        from armhost.ui.scope import decimate_minmax
        x = np.arange(10.0)
        y = np.ones((10, 3))
        x2, y2 = decimate_minmax(x, y, 500)
        self.assertIs(x2, x)


class TestMonitorWindow(unittest.TestCase):
    def setUp(self):
        self.w = MonitorWindow()
        self.w.resize(1500, 920)
        self.w.show()

    def tearDown(self):
        self.w.close()
        pump(0.1)

    def test_live_mock(self):
        w = self.w
        w._switch(MockBackend(rate_hz=500))
        pump(1.5)
        self.assertGreater(w.ctrl.buffer.size, 400)
        # 默认勾选的曲线有数据
        x, y = w.scope.curves[(0, "angle_deg")].getData()
        self.assertIsNotNone(x)
        self.assertGreater(len(x), 100)
        # 未勾选的通道没有数据，也不可见
        self.assertFalse(w.scope.curves[(0, "torque_cmd_nm")].isVisible())
        self.assertNotEqual(w.table.item(0, 0).text(), "—")
        self.assertGreater(w.scope.frames, 20)            # 30 Hz 定时刷新在跑
        self.assertIn("Hz", w.status_panel.time.v["mcu_rate"].text())

        # 暂停：显示冻结，数据仍在进缓存
        w.scope.set_paused(True)
        n0 = len(w.scope.curves[(0, "angle_deg")].getData()[0])
        size0 = w.ctrl.buffer.total
        pump(0.3)
        self.assertEqual(len(w.scope.curves[(0, "angle_deg")].getData()[0]), n0)
        self.assertGreater(w.ctrl.buffer.total, size0)
        w.scope.set_paused(False)

        # 切换窗口长度 / 时间轴 / 通道
        w.scope.win_combo.setCurrentIndex(0)
        w.scope.base_combo.setCurrentIndex(1)
        w.scope.selector.toggle_column("torque_cmd_nm")
        pump(0.2)
        self.assertTrue(w.scope.curves[(3, "torque_cmd_nm")].isVisible())

        # 统计页
        w.tabs.setCurrentWidget(w.stats)
        pump(0.6)
        self.assertGreater(w.stats.rowCount(), 0)

        # 急停（模拟设备支持）
        w.estop_btn.click()
        pump(0.3)
        self.assertIn("急停", w.events.toPlainText())

        (ROOT / "logs").mkdir(exist_ok=True)
        w.grab().save(str(ROOT / "logs" / "ui_smoke.png"))

    def _fill(self, n=2000):
        from armhost.model.state import RobotSample
        from armhost.protocol import JointRecord
        for i in range(n):        # J{j} angle = i + j，便于核对读数
            self.w.ctrl.buffer.append(RobotSample(i * 5000, i * 5_000_000, i, 1, 3, 0,
                                                  tuple(JointRecord(angle_deg=float(i + j), torque_nm=0.01 * j)
                                                        for j in range(7))))
        self.w.scope.refresh(force=True)
        pump(0.1)

    def test_crosshair_reads_raw_value(self):
        s = self.w.scope
        self._fill()
        s.set_paused(True)
        s._mouse_x = -1.0                 # 最新样本前 1 s = 第 1999-200 = 1799 个样本
        s._update_crosshair()
        ro = s.readouts["angle"]
        self.assertTrue(ro.isVisible())
        html = ro.textItem.toHtml()
        self.assertIn("1799.0000", html)   # J0 角度
        self.assertIn("1805.0000", html)   # J6 角度
        self.assertTrue(s.vlines["torque"].isVisible())
        self.assertAlmostEqual(s.vlines["angle"].value(), -1.0, places=6)
        # 力矩图只列力矩曲线
        self.assertNotIn("角度", s.readouts["torque"].textItem.toHtml().replace("角度 (deg)", ""))
        # 关掉十字光标
        s.cross_btn.setChecked(False)
        self.assertFalse(ro.isVisible())

    def test_solo_plot(self):
        s = self.w.scope
        self._fill()
        h_all = s.plots["torque"].getViewBox().height()
        s.set_solo("torque")
        pump(0.2)
        self.assertFalse(s.plots["angle"].isVisible())
        self.assertGreater(s.plots["torque"].getViewBox().height(), h_all * 2)
        self.assertTrue(s.plots["torque"].getAxis("bottom").style["showValues"])
        self.assertGreater(len(s.curves[(0, "torque_nm")].getData()[0]), 100)
        s.set_solo(None)
        pump(0.2)
        self.assertTrue(s.plots["angle"].isVisible())
        self.assertFalse(s.plots["angle"].getAxis("bottom").style["showValues"])
        self.assertEqual(s.solo_combo.currentIndex(), 0)
        s.set_solo("angle")
        pump(0.2)
        self.assertGreater(s.plots["angle"].getViewBox().height(), h_all * 2)   # 第二次放大也要占满
        s._mouse_x = -0.5
        s._update_crosshair()
        self.assertFalse(s.readouts["torque"].isVisible())
        self.assertTrue(s.readouts["angle"].isVisible())
        (ROOT / "logs").mkdir(exist_ok=True)
        self.w.grab().save(str(ROOT / "logs" / "ui_solo.png"))

    def test_refresh_cost(self):
        """一次刷新（全部 35 条曲线、60 s @ 500 Hz 缓存）的耗时，保证不拖慢 GUI"""
        w = self.w
        from armhost.model.state import RobotSample
        from armhost.protocol import JointRecord
        for i in range(30000):
            w.ctrl.buffer.append(RobotSample(i * 2000, i * 2_000_000, i, 1, 3, 0,
                                             tuple(JointRecord(angle_deg=float(i % 360)) for _ in range(7))))
        for q in ("target_deg", "torque_cmd_nm"):
            w.scope.selector.toggle_column(q)
        w.scope.win_combo.setCurrentIndex(5)      # 60 s
        pump(0.2)
        t0 = time.perf_counter()
        for _ in range(10):
            w.scope.refresh(force=True)
            QCoreApplication.processEvents()
        dt = (time.perf_counter() - t0) / 10
        print(f"\n  refresh 35 curves x 30000 pts: {dt * 1e3:.1f} ms/frame")
        self.assertLess(dt, 0.1)


if __name__ == "__main__":
    unittest.main()
