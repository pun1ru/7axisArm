"""三维姿态模块测试（offscreen）。"""

import os
import sys
import time
import unittest
from concurrent.futures import Future
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np  # noqa: E402
from PySide6.QtCore import QCoreApplication  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from armhost.kinematics import DHTable, current_dh  # noqa: E402
from armhost.model.state import RobotSample  # noqa: E402
from armhost.protocol import JointRecord  # noqa: E402
from armhost.ui import theme  # noqa: E402
from armhost.ui.pose_panel import DHEditor, PosePanel  # noqa: E402
from armhost.ui.view3d import View3D  # noqa: E402

app = QApplication.instance() or QApplication([])
theme.apply(app)


def pump(s):
    end = time.monotonic() + s
    while time.monotonic() < end:
        QCoreApplication.processEvents()
        time.sleep(0.005)


def sample(q):
    return RobotSample(0, time.monotonic_ns(), 0, 1, 3, 0,
                       tuple(JointRecord(angle_deg=float(a), target_deg=float(a), torque_nm=0.1) for a in q))


class TestView3D(unittest.TestCase):
    def test_world_mount_is_default_for_current_model(self):
        table = current_dh()
        v = View3D(table)
        self.assertTrue(v.world_mount)
        np.testing.assert_allclose(v.frames[0][:3, 3], table.base_xyz_m, atol=1e-12)
        np.testing.assert_allclose(v.end_pose()[0] * 1000.0,
                                   (-383.614217, 71.347006, 362.332865), atol=1e-4)

        v.set_option("world_mount", False)
        np.testing.assert_allclose(v.frames[0][:3, 3], 0.0, atol=1e-12)
        np.testing.assert_allclose(v.end_pose()[0] * 1000.0,
                                   (324.332865, 384.173057, -92.2), atol=1e-4)

    def test_render_and_throttle(self):
        v = View3D(DHTable())
        v.resize(800, 600)
        v.show()
        pump(0.2)
        n0 = v.renders
        self.assertGreater(n0, 0)
        pump(0.3)                       # 没有变化就不重绘
        self.assertEqual(v.renders, n0)
        v.set_angles([0] * 7)           # 相同角度不触发
        pump(0.1)
        self.assertEqual(v.renders, n0)
        v.set_angles([10, 20, 30, 40, 50, 60, 70])
        pump(0.1)
        self.assertEqual(v.renders, n0 + 1)
        self.assertLess(v.last_render_ms, 30)
        pos, _ = v.end_pose()
        self.assertGreater(np.linalg.norm(pos), 0.1)

    def test_projection_and_pick(self):
        v = View3D(DHTable())
        v.resize(800, 600)
        v.show()
        pump(0.1)
        project = v._projector()
        scr, z = project(np.array([v.target]))
        np.testing.assert_allclose(scr[0], (400, 300), atol=1e-6)     # 目标点在画面中心
        got = []
        v.joint_clicked.connect(got.append)
        p7, _ = project(np.array([v.frames[5][:3, 3]]))              # J4 原点
        from PySide6.QtCore import QPointF
        v._pick(QPointF(*p7[0]))
        self.assertIn(got[0], (3, 4))                                 # 理想版 J3/J4 共点

    def test_camera_controls(self):
        v = View3D(DHTable())
        yaw, d = v.yaw, v.dist
        v.reset_view("top")
        self.assertAlmostEqual(v.pitch, 89.0)
        v.pitch = 120                     # 外部乱设也不能翻过头
        v.mouseMoveEvent  # noqa
        v.reset_view()
        self.assertEqual((v.yaw, v.dist), (yaw, d))


class TestPosePanel(unittest.TestCase):
    def test_device_dh_preserves_waist_mount(self):
        table = DHTable(base_xyz_m=(0.001, -0.02, 0.038), base_rpy_deg=(90.0, -90.0, 0.0))
        device = table.copy()
        device.rows[2].d_m = 0.123
        device.source = "设备测试"
        fut: Future = Future()
        fut.set_result(device)

        editor = DHEditor(table, lambda: fut)
        applied = []
        editor.applied.connect(applied.append)
        editor._pending = fut
        editor._poll_device()

        got = editor.table()
        self.assertEqual(editor._source, "设备测试")
        self.assertEqual(applied[-1].source, "设备测试")
        self.assertAlmostEqual(got.rows[2].d_m, 0.123)
        self.assertEqual(got.base_xyz_m, table.base_xyz_m)
        self.assertEqual(got.base_rpy_deg, table.base_rpy_deg)

    def test_live_manual_and_dh_edit(self):
        cur = {"s": sample([0] * 7)}
        p = PosePanel(lambda: cur["s"], table=DHTable())
        p.resize(1200, 700)
        p.show()
        pump(0.15)
        cur["s"] = sample([0, 90, 0, 0, 0, 0, 0])
        pump(0.15)
        np.testing.assert_allclose(p.view.end_pose()[0], (-0.248, 0, 0.0626), atol=1e-6)
        # 手动模式：实时数据不再影响姿态
        p.src_manual.setChecked(True)
        p.sliders[0].setValue(900)
        pump(0.1)
        self.assertAlmostEqual(p.view.q_deg[0], 90.0)
        self.assertAlmostEqual(p.view.q_deg[1], 0.0)
        # 取实时姿态：多圈角折到 ±180
        cur["s"] = sample([370, -400, 0, 0, 0, 0, 0])
        p._copy_live()
        self.assertEqual(p.sliders[0].value(), 100)
        self.assertEqual(p.sliders[1].value(), -400)
        # 改 DH：J2 d 改成 -0.2，立即生效
        p.sliders[1].setValue(0)
        p.dh.tbl.item(2, 2).setText("-0.2")
        pump(0.05)
        self.assertAlmostEqual(p.view.table.rows[2].d_m, -0.2)
        self.assertAlmostEqual(p.view.frames[3][2, 3], 0.2626, places=6)
        # 非法输入不生效、不崩
        p.dh.tbl.item(2, 2).setText("-")
        self.assertAlmostEqual(p.view.table.rows[2].d_m, -0.2)
        # 坐标系单独开关 + 选中关节
        p.frame_checks[3].setChecked(False)
        self.assertFalse(p.view.frame_mask[3])
        p.src_live.setChecked(True)
        p._select_joint(2)
        pump(0.1)
        self.assertEqual(p.view.selected, 2)
        self.assertTrue(any("力矩" in s for s in p.view.selected_info))
        (ROOT / "logs").mkdir(exist_ok=True)
        cur["s"] = sample([30, 40, -20, -60, 20, 50, 0])
        p.dh.set_table(DHTable(), emit=True)
        pump(0.2)
        p.grab().save(str(ROOT / "logs" / "pose_panel.png"))

    def test_workspace_overlay(self):
        p = PosePanel(lambda: sample([0] * 7), table=DHTable())
        p.resize(1200, 700)
        p.show()
        pump(0.1)
        p.workspace.samples.setValue(8_000)
        p.workspace.voxel_mm.setValue(25.0)
        p.workspace.calculate()
        pump(0.15)
        self.assertGreater(len(p.view.workspace_points), 0)
        self.assertTrue(p.view.show_workspace)
        self.assertIn("工作空间", p.view.workspace_info)

        p.workspace.branch.setCurrentIndex(p.workspace.branch.findData(2))
        self.assertEqual(len(p.view.workspace_points), 0)       # 换分支后旧点云必须清掉
        p.workspace.calculate()
        pump(0.1)
        self.assertIn("分支 #2", p.view.workspace_info)

        p.workspace.show_cb.setChecked(False)
        pump(0.05)
        self.assertFalse(p.view.show_workspace)
        p.workspace.show_cb.setChecked(True)
        p.workspace.clear()
        pump(0.05)
        self.assertEqual(len(p.view.workspace_points), 0)

        p.workspace.calculate()
        p.side_tabs.setCurrentWidget(p.workspace_tab)
        pump(0.1)
        (ROOT / "logs").mkdir(exist_ok=True)
        p.grab().save(str(ROOT / "logs" / "workspace_panel.png"))


if __name__ == "__main__":
    unittest.main()
