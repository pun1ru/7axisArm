"""DH 正运动学测试：与固件约定、dh_params.md 第 7 节理想帧逐项核对。"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from armhost.kinematics import (ARMSIM_MAIN, BIGARM_BASE_RPY_DEG, BIGARM_BASE_XYZ_M, BIGARM_DH,  # noqa: E402
                                BUILTIN_DH, FIRMWARE_MODULE_INIT, DHRow, DHTable, current_dh,
                                default_dh, euler_zyx_deg, forward_frames, link_transform)


class TestDH(unittest.TestCase):
    def test_firmware_parse_matches_builtin(self):
        if not FIRMWARE_MODULE_INIT.exists():
            self.skipTest("没有单片机工程")
        t = DHTable.from_firmware_source()
        self.assertEqual(t.n, 7)
        self.assertEqual([r.as_tuple() for r in t.rows], [r.as_tuple() for r in BUILTIN_DH],
                         "固件 DH 改了，同步 armhost/kinematics/dh.py 的 BUILTIN_DH")
        self.assertIn("module_init", default_dh().source)

    def test_parse_variants(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "m.c"
            p.write_text("#define ARM_DH_J0   {0.0f, 0.0f, 0.0626f, 0.0f}\n"
                         "#define ARM_DH_J1 { -90.0f,1e-3f , -0.148f,  5 }\n", encoding="utf-8")
            t = DHTable.from_firmware_source(p)
            self.assertEqual(t.rows[1].as_tuple(), (-90.0, 0.001, -0.148, 5.0))
            p.write_text("#define ARM_DH_J1 {0,0,0,0}\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                DHTable.from_firmware_source(p)

    def test_current_bigarm_dh_and_world_frames(self):
        if not ARMSIM_MAIN.exists():
            self.skipTest("没有 ArmSim 源码")
        t = DHTable.from_armsim_source()
        self.assertEqual([r.as_tuple() for r in t.rows], [r.as_tuple() for r in BIGARM_DH],
                         "main.cpp 的 DH 改了，同步 armhost/kinematics/dh.py 的 BIGARM_DH")
        np.testing.assert_allclose(t.base_xyz_m, BIGARM_BASE_XYZ_M, atol=1e-12)
        np.testing.assert_allclose(t.base_rpy_deg, BIGARM_BASE_RPY_DEG, atol=1e-6)
        self.assertEqual(t.source, "ArmSim BigArm main.cpp")

        expected_world_mm = np.array([
            [0.558840, -20.852994, 38.000000],
            [0.558840, 68.847006, 448.500000],
            [0.558840, 68.847006, 448.500000],
            [-178.751984, 68.847006, 392.005501],
            [-178.751984, 71.347006, 392.005501],
            [-383.614217, 71.347006, 362.332865],
            [-383.614217, 71.347006, 362.332865],
            [-383.614217, 71.347006, 362.332865],
        ])
        frames = forward_frames(t, [0.0] * t.n)
        world_mm = np.array([(t.base_transform @ f)[:3, 3] * 1000.0 for f in frames])
        np.testing.assert_allclose(world_mm, expected_world_mm, atol=1e-4)
        self.assertEqual(current_dh().source, t.source)

    def test_zero_pose_matches_dh_params_md(self):
        """dh_params.md 7.3：理想版 O_1=O_2 在 +62.6 mm，O_3=O_4 在 +210.6 mm，O_5=O_6=O_7 在 +310.6 mm（相对 O_0）
        7 节轴向：z1=+Z z2=−Y z3=−Z z4=+Y z5=+Z z6=−Y z7=−Z"""
        F = forward_frames(DHTable(), [0] * 7)
        z_expect = [0.0626, 0.0626, 0.2106, 0.2106, 0.3106, 0.3106, 0.3106]
        axis_expect = [(0, 0, 1), (0, -1, 0), (0, 0, -1), (0, 1, 0), (0, 0, 1), (0, -1, 0), (0, 0, -1)]
        for i in range(7):
            np.testing.assert_allclose(F[i + 1][:3, 3], (0, 0, z_expect[i]), atol=1e-9)
            np.testing.assert_allclose(F[i + 1][:3, 2], axis_expect[i], atol=1e-9)

    def test_rotation_about_joint_axis(self):
        """J0 转 90°：J0 轴不动，后面所有原点都在 J0 轴上（理想版全共线），x 轴转到 +Y"""
        F = forward_frames(DHTable(), [90, 0, 0, 0, 0, 0, 0])
        np.testing.assert_allclose(F[1][:3, 0], (0, 1, 0), atol=1e-9)
        np.testing.assert_allclose(F[7][:3, 3], (0, 0, 0.3106), atol=1e-9)
        # J1 转 90°：末端绕 −Y（z2）转，从 +Z 方向转到 −X
        F = forward_frames(DHTable(), [0, 90, 0, 0, 0, 0, 0])
        np.testing.assert_allclose(F[7][:3, 3], (-0.248, 0, 0.0626), atol=1e-9)

    def test_multiturn_equivalent(self):
        a = forward_frames(DHTable(), [10, 20, 30, 40, 50, 60, 70])[-1]
        b = forward_frames(DHTable(), [370, 20 - 720, 30, 40, 50, 60, 70])[-1]
        np.testing.assert_allclose(a, b, atol=1e-9)

    def test_matches_firmware_formula(self):
        r = DHRow(30.0, 0.05, 0.02, 10.0)
        T = link_transform(r, 20.0)
        th, al = np.radians(30.0), np.radians(30.0)
        ref = np.array([[np.cos(th), -np.sin(th), 0, 0.05],
                        [np.sin(th) * np.cos(al), np.cos(th) * np.cos(al), -np.sin(al), -np.sin(al) * 0.02],
                        [np.sin(th) * np.sin(al), np.cos(th) * np.sin(al), np.cos(al), np.cos(al) * 0.02],
                        [0, 0, 0, 1]])
        np.testing.assert_allclose(T, ref, atol=1e-12)
        np.testing.assert_allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-12)

    def test_euler(self):
        F = forward_frames(DHTable(), [0] * 7)
        roll, pitch, yaw = euler_zyx_deg(F[-1][:3, :3])
        # q=0 末端 z7 = −Z：绕 x 转了 180°
        self.assertAlmostEqual(abs(roll), 180.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)

    def test_json_roundtrip(self):
        t = DHTable()
        t.rows[3].d_m = 0.123
        t.base_xyz_m = (0.001, -0.002, 0.038)
        t.base_rpy_deg = (90.0, -90.0, 0.0)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "dh.json"
            t.save_json(p)
            raw = json.loads(p.read_text(encoding="utf-8"))
            self.assertIn("rows", raw)
            self.assertEqual(raw["base"]["xyz_m"], list(t.base_xyz_m))
            loaded = DHTable.load_json(p)
            self.assertEqual(loaded.rows[3].d_m, 0.123)
            self.assertEqual(loaded.base_xyz_m, t.base_xyz_m)
            self.assertEqual(loaded.base_rpy_deg, t.base_rpy_deg)


if __name__ == "__main__":
    unittest.main()
