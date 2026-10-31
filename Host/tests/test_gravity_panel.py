"""上位机「重力拟合」标签页：与命令行同一份算法，选项传递、错误处理、与主窗口集成。"""

import contextlib
import io
import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np  # noqa: E402
from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

import gravity_fit as cli  # noqa: E402
from armhost.kinematics import DHTable  # noqa: E402
from armhost.model import gravity_fit as gf  # noqa: E402
from armhost.ui import theme  # noqa: E402
from armhost.ui.gravity_panel import GravityFitPanel  # noqa: E402
from test_gravity_fit import _poses, _write_csv  # noqa: E402

app = QApplication.instance() or QApplication([])
theme.apply(app)
TABLE = DHTable()


def _synthetic(d: Path, name: str, rng, pi, n_pose=25, **kw) -> Path:
    q = _poses(rng, n_pose, 170)
    tau = [gf.gravity_regressor(TABLE, np.radians(r)) @ pi for r in q]
    p = d / name
    _write_csv(p, q, tau, **kw)
    return p


class TestGravityPanel(unittest.TestCase):
    def setUp(self):
        self.panel = GravityFitPanel(lambda: TABLE)
        self.panel.files.clear()          # 不依赖 logs/gravity 里碰巧有什么
        self.rng = np.random.default_rng(7)
        self.pi = self.rng.normal(scale=0.05, size=28)
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_fit_checked_files_only(self):
        a = _synthetic(self.dir, "a.csv", self.rng, self.pi)
        b = _synthetic(self.dir, "b.csv", self.rng, self.pi)
        c = _synthetic(self.dir, "c.csv", self.rng, self.pi)
        for p in (a, b, c):
            self.assertTrue(self.panel.add_file(p))
        self.assertFalse(self.panel.add_file(a), "重复添加应忽略")
        self.panel.files.item(2).setCheckState(Qt.Unchecked)
        r = self.panel.run()
        self.assertIsNotNone(r)
        self.assertEqual([d.name for d in r.datasets], ["a.csv", "b.csv"])
        self.assertEqual(r.rank, 12)
        self.assertLess(r.rmse, 1e-9)
        self.assertEqual(len(r.cv), 2)
        self.assertIn("秩 12 / 28", self.panel.report.toPlainText())
        self.assertTrue(self.panel.save_btn.isEnabled())
        self.assertTrue(self.panel.copy_btn.isEnabled())

    def test_matches_cli(self):
        files = [_synthetic(self.dir, f"{k}.csv", self.rng, self.pi) for k in range(2)]
        for p in files:
            self.panel.add_file(p)
        r = self.panel.run()
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            self.assertEqual(cli.main([*map(str, files), "--dh", "builtin"]), 0)
        self.assertEqual(self.panel.report.toPlainText().strip(), buf.getvalue().strip())
        self.assertEqual(r.rank, 12)

    def test_options_are_forwarded(self):
        self.panel.torque.setCurrentIndex(1)
        self.panel.sign.setCurrentIndex(1)
        self.panel.rcond.setCurrentIndex(0)
        self.panel.bias.setChecked(True)
        self.panel.states.setText("5, 6")
        self.panel.vel_on.setChecked(True)
        self.panel.vel.setValue(15)
        self.panel.cv.setChecked(False)
        o = self.panel.options()
        self.assertEqual((o.torque_col, o.sign, o.rcond, o.bias, o.arm_states, o.max_vel, o.cv),
                         ("cmd", -1.0, 1e-2, True, [5, 6], 15.0, False))
        self.panel.vel_on.setChecked(False)
        self.panel.states.setText("")
        o = self.panel.options()
        self.assertIsNone(o.max_vel)
        self.assertIsNone(o.arm_states)

    def test_state_and_velocity_filters(self):
        n = 8
        q = _poses(self.rng, n, 170)
        tau = [gf.gravity_regressor(TABLE, np.radians(r)) @ self.pi for r in q]
        vel = np.zeros((n, 7))
        vel[0, 3] = 90
        p = self.dir / "m.csv"
        _write_csv(p, q, tau, vel, state=[6, 6, 6, 6, 5, 5, 6, 6])
        self.panel.add_file(p)
        self.panel.states.setText("6")
        self.panel.vel_on.setChecked(True)
        r = self.panel.run()
        self.assertEqual(r.n_poses, 5)        # 8 - 2(模式) - 1(运动)

    def test_errors_do_not_raise(self):
        self.panel.run()
        self.assertIn("没有勾选", self.panel.report.toPlainText())
        self.assertFalse(self.panel.save_btn.isEnabled())
        bad = self.dir / "bad.csv"
        bad.write_text("a,b\n1,2\n", encoding="utf-8")
        self.panel.add_file(bad)
        self.assertIsNone(self.panel.run())
        self.assertIn("错误", self.panel.report.toPlainText())
        self.assertFalse(self.panel.copy_btn.isEnabled())
        self.panel.files.clear()
        self.panel.add_file(self.dir / "missing.csv")
        self.assertIsNone(self.panel.run())

    def test_uses_current_dh(self):
        p = _synthetic(self.dir, "d.csv", self.rng, self.pi)
        self.panel.add_file(p)
        t = TABLE.copy()
        # 注意不能改 d：沿关节轴的偏移会被质心参数 h 吸收，拟合对 d 不敏感。改扭角才真正改变几何。
        t.rows[3].alpha_deg = 60.0
        t.source = "测试DH"
        self.panel._get_table = lambda: t
        r = self.panel.run()
        self.assertIn("测试DH", self.panel.report.toPlainText())
        self.assertGreater(r.rmse, 1e-4, "扭角变了，用另一份 DH 生成的数据不该再被精确拟合")


class TestMainWindowIntegration(unittest.TestCase):
    def test_tab_present_and_uses_pose_dh(self):
        from armhost.ui.main_window import MonitorWindow
        w = MonitorWindow()
        try:
            i = w.tabs.indexOf(w.gravity_tab)           # 面板包在滚动区里，小窗口下不撑大窗口
            self.assertGreaterEqual(i, 0)
            self.assertEqual(w.tabs.tabText(i), "重力拟合")
            self.assertIs(w.gravity_tab.widget(), w.gravity)
            self.assertIs(w.gravity._get_table(), w.pose.view.table)
        finally:
            w.close()


if __name__ == "__main__":
    unittest.main()
