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

from PySide6.QtCore import QCoreApplication, Qt  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from armhost.model.backend import MockBackend  # noqa: E402
from armhost.protocol import ArmState  # noqa: E402
from armhost.ui import theme  # noqa: E402
from armhost.ui.main_window import MonitorWindow  # noqa: E402

app = QApplication.instance() or QApplication([])
theme.apply(app)


def pump(seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        QCoreApplication.processEvents()
        time.sleep(0.005)


def wait_until(pred, timeout: float = 3.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        pump(0.02)
    return False


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


class TestPlotCompat(unittest.TestCase):
    def test_curve_bounding_rect_skips_pixel_vectors(self):
        import pyqtgraph as pg
        from armhost.ui import scope  # noqa: F401  安装兼容补丁

        curve = pg.PlotCurveItem([0.0, 1.0], [0.0, 1.0], pen=pg.mkPen(width=1))
        curve.pixelVectors = lambda *args, **kwargs: self.fail("不应进入 Point(QPointF) 路径")
        self.assertEqual(curve.pixelPadding(), 0.0)
        curve.boundingRect()


class TestWindowLayout(unittest.TestCase):
    """小窗口可用：最小尺寸不能把窗口撑出常见屏幕，分隔条要能拖、不能把某块拖没。"""

    def setUp(self):
        self.w = MonitorWindow()
        self.w.show()
        pump(0.2)

    def tearDown(self):
        self.w.close()
        pump(0.1)

    def test_minimum_size_fits_small_screens(self):
        h = self.w.minimumSizeHint()
        # 1366x768 笔记本去掉任务栏约 1366x728；之前被撑到 1614x1086，工具区在屏幕外点不到
        self.assertLessEqual(h.width(), 1100, f"最小宽度 {h.width()} 太大，某个面板又把窗口撑宽了")
        self.assertLessEqual(h.height(), 700, f"最小高度 {h.height()} 太大，某个面板又把窗口撑高了")

    def test_tool_dock_reachable_in_small_window(self):
        w = self.w
        for size in ((1366, 700), (1000, 620)):
            w.resize(*size)
            pump(0.2)
            r = w.tool_dock.geometry()
            self.assertLessEqual(r.bottom(), w.height(), f"{size}: 工具区溢出窗口 {r.getRect()}")
            self.assertGreaterEqual(r.height(), 100)
            self.assertLessEqual(r.width(), w.width())

    def test_initial_size_clamped_to_screen(self):
        from PySide6.QtGui import QGuiApplication
        g = QGuiApplication.primaryScreen().availableGeometry()
        self.assertLessEqual(self.w.width(), max(g.width(), self.w.minimumSizeHint().width()))
        self.assertLessEqual(self.w.height(), max(g.height(), self.w.minimumSizeHint().height()))

    def test_splitters_drag_and_collapse_and_restore(self):
        w = self.w
        w.resize(1200, 760)
        pump(0.3)
        sp = w.center_split
        total = sum(sp.sizes())
        self.assertTrue(sp.childrenCollapsible(), "折叠到 0 的行为要保留")
        sp.moveSplitter(total, 1)                   # 关节表折叠到 0
        pump(0.1)
        self.assertEqual(sp.sizes()[1], 0)
        self.assertGreaterEqual(sp.handle(1).height(), 4, "折叠后把手必须仍可见可抓")
        sp.moveSplitter(total - 150, 1)             # 从折叠状态拖回来
        pump(0.1)
        self.assertGreater(sp.sizes()[1], 0)
        sp.moveSplitter(0, 1)                       # 标签页折叠到 0 也能拖回
        pump(0.1)
        self.assertEqual(sp.sizes()[0], 0)
        sp.moveSplitter(total // 2, 1)
        pump(0.1)
        self.assertGreater(sp.sizes()[1], 100, "关节表应该能被拉高")

        # 底部工具区、右栏宽度、右栏上下分隔都能调
        h0 = w.tool_dock.height()
        w.resizeDocks([w.tool_dock], [h0 + 80], Qt.Vertical)
        pump(0.1)
        self.assertGreater(w.tool_dock.height(), h0)
        w0 = w.status_dock.width()
        w.resizeDocks([w.status_dock], [w0 + 100], Qt.Horizontal)
        pump(0.1)
        self.assertGreater(w.status_dock.width(), w0)

    def test_default_layout_not_cramped(self):
        w = self.w
        w.resize(1500, 900)
        pump(0.4)
        self.assertGreaterEqual(w.tool_dock.height(), 180, "默认工具区太矮，重力拟合报告看不了")
        self.assertGreaterEqual(w.table.height(), 22 * 7, "默认关节表应显示全部 7 行")


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

    def test_host_panel_reads_limits_and_sends_target(self):
        w = self.w
        backend = MockBackend(rate_hz=200)
        w._switch(backend)
        self.assertTrue(wait_until(lambda: w.host._limits_ready))
        self.assertTrue(w.host.spins[0].isEnabled())
        self.assertEqual(w.host.spins[0].minimum(), -180.0)
        self.assertEqual(w.host.spins[0].maximum(), 180.0)
        backend.device.arm_state = ArmState.HOST
        pump(0.4)
        self.assertTrue(wait_until(lambda: w.host.send_closed_btn.isEnabled()))

        w.host.joint_combo.setCurrentIndex(0)
        w.host.spins[0].setValue(12.0)
        w.host.send_closed_btn.click()
        self.assertTrue(wait_until(lambda: backend.device.host_closed_loop_mask == 1))
        self.assertEqual(backend.device.host_target_valid_mask, 1)
        self.assertEqual(backend.device.joints[0].host_target, 12.0)

        w.host.joint_combo.setCurrentIndex(1)
        w.host.spins[1].setValue(20.0)
        self.assertTrue(wait_until(lambda: w.host.send_open_btn.isEnabled()))
        w.host.send_open_btn.click()
        self.assertTrue(wait_until(lambda: backend.device.joints[1].host_target == 20.0))
        self.assertEqual(backend.device.host_closed_loop_mask, 1)   # 只动 J0 闭环
        self.assertEqual(backend.device.joints[6].host_target, 0.0) # 未发送关节不变

        self.assertTrue(wait_until(lambda: w.host.stop_btn.isEnabled()))
        w.host.stop_btn.click()
        self.assertTrue(wait_until(lambda: backend.device.host_closed_loop_mask == 0))
        self.assertTrue(wait_until(lambda: "0/7" in w.host.state_label.text(), 5.0))

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
