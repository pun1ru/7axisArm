"""固件 Arm_Run 的行为测试：把 arm.c / arm_jointctrl.c / arm_kinematics.c / arm_dynamics.c / dh.c / pid.c / algorism.c
用主机 C 编译器真实编译（电机与 CAN 用桩），捕获 sendcommand 时的力矩，验证：

  ENABLE：每个关节 = PID(目标跟随观测，误差 0) + 重力前馈；
  HOST ：重力补偿遵从 ENABLE 的逻辑，所有关节都参与——闭环关节 = PID(追上位机目标) + 重力，开环关节 = 只有重力；
  output_max 只限制 PID，重力不受它限制；两部分相加后只受 total_output_max 限幅；
  关掉总开关 / 某轴掩码 / 模型无效时对应关节不带重力；
  PROTECTED 一律零指令，不带重力；ARM_GRAVITY_OUTPUT=0 时 HOST 开环关节回到零力矩。
没有主机 C 编译器时跳过。
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from armhost.kinematics import DHTable  # noqa: E402
from armhost.model.gravity_fit import gravity_regressor  # noqa: E402
from test_c_gravity import PI_USER, _find_vcvars  # noqa: E402

FW = ROOT.parent / "Real" / "DIYcontroller" / "LibSE"
STUB = ROOT / "tests" / "c_stub"
DRIVER = ROOT / "tests" / "c_arm_driver.c"
SRC = [FW / "Module" / "Arm" / n for n in ("arm.c", "arm_jointctrl.c", "arm_kinematics.c", "arm_dynamics.c")] + [
    FW / "Algorithm" / "Kinematics" / "dh.c", FW / "Algorithm" / "Control" / "PID" / "pid.c",
    FW / "Algorithm" / "Math" / "algorism.c", STUB / "motor_stub.c", DRIVER]
INC = [STUB, FW / "Module" / "Arm", FW / "Algorithm" / "Kinematics", FW / "Algorithm" / "Math",
       FW / "Algorithm" / "Control" / "PID", FW / "Drivers" / "Motor"]

PROTECTED, ENABLE, HOST = 0, 1, 6


def build(out: Path, output_flag: int = 1) -> bool:
    for cc in ("gcc", "clang"):
        exe = shutil.which(cc)
        if exe:
            cmd = [exe, "-std=gnu99", "-O1", f"-DARM_GRAVITY_OUTPUT={output_flag}", *[f"-I{i}" for i in INC],
                   *map(str, SRC), "-lm", "-o", str(out)]
            return subprocess.run(cmd, capture_output=True).returncode == 0
    vc = _find_vcvars()
    if vc is None:
        return False
    flags = " ".join(f'/I"{i}"' for i in INC)
    files = " ".join(f'"{s}"' for s in SRC)
    bat = (f'call "{vc}" >nul && cl /nologo /O1 /W3 /std:c11 /D_CRT_SECURE_NO_WARNINGS /DARM_GRAVITY_OUTPUT={output_flag} '
           f'{flags} {files} /Fe:"{out}" /Fo:"{out.parent}\\\\"')
    r = subprocess.run(bat, shell=True, capture_output=True, text=True, encoding="utf-8", errors="replace", cwd=out.parent)
    if r.returncode != 0:
        print(r.stdout[-3000:], r.stderr[-1000:])
    return r.returncode == 0


class Sim:
    """一次驱动会话：喂命令，读 run 的输出"""

    def __init__(self, exe: Path, en=1, use_beta=1, sign=1, mask=0x7F, kp=0.0, omax=2.5, tmax=None):
        if tmax is None:
            tmax = omax
        self.cmds = [f"cfg {en} {use_beta} {sign} {mask} {kp!r} {omax!r} {tmax!r}"]
        self.out_lines = []
        self.expect = []

    def add(self, *lines, expect=None):
        self.cmds.extend(lines)

    def run(self, exe) -> list:
        r = subprocess.run([str(exe)], input="\n".join(self.cmds) + "\n", capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=30)
        assert r.returncode == 0, r.stderr
        return r.stdout.strip().splitlines()


def parse_run(line: str):
    a, b, c = line.split("|")
    return np.array(a.split(), float), np.array(b.split(), float), [int(x) for x in c.split()]


def q_line(q):
    return "q " + " ".join(f"{float(v):.9g}" for v in q)


def gravity_ref(q_deg):
    return gravity_regressor(DHTable(), np.radians(q_deg)) @ PI_USER


Q = [0.0, -30.0, 40.0, -50.0, 20.0, 10.0, 5.0]      # 限位内的一般位形


class TestArmRunGravity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not all(p.exists() for p in SRC):
            raise unittest.SkipTest("缺固件源码")
        cls.tmp = tempfile.TemporaryDirectory()
        d = Path(cls.tmp.name)
        cls.exe = d / "arm_on.exe"
        cls.exe_off = d / "arm_off.exe"
        (d / "on").mkdir(); (d / "off").mkdir()
        if not build(d / "on" / "arm.exe", 1):
            raise unittest.SkipTest("没有可用的主机 C 编译器，或桩编译失败")
        cls.exe = d / "on" / "arm.exe"
        cls.have_off = build(d / "off" / "arm.exe", 0)
        cls.exe_off = d / "off" / "arm.exe"

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    # ---- helpers
    def session(self, lines, **kw):
        s = Sim(self.exe, **kw)
        s.add(*lines)
        return s.run(self.exe)

    def test_protected_never_outputs_gravity(self):
        out = self.session([q_line(Q), "run"])
        tau, cmd, flags = parse_run(out[0])
        self.assertTrue(np.all(tau == 0.0), f"PROTECTED 必须零指令：{tau}")
        self.assertEqual(flags[0], 1, "模型照常计算（只算不出）")
        self.assertTrue(np.any(cmd != 0.0))

    def test_enable_is_pid_zero_error_plus_gravity(self):
        ref = gravity_ref(Q)
        out = self.session(["state 1", q_line(Q), "run", "run"], kp=0.5, omax=5.0)
        tau, cmd, flags = parse_run(out[1 + 1])          # state 的 ok 行占一行
        np.testing.assert_allclose(tau[1:], ref[1:], atol=1e-5)
        self.assertEqual(tau[0], 0.0)
        np.testing.assert_allclose(cmd, tau, atol=1e-7)
        self.assertEqual(flags, [1, 1])

    def test_host_open_loop_joints_get_gravity_only(self):
        """用户要求：HOST 下也遵从 enable 的补偿逻辑，开环关节不再是零力矩，而是只有重力"""
        ref = gravity_ref(Q)
        out = self.session(["state 6", q_line(Q), "run", "run"], kp=0.5, omax=5.0)
        tau, cmd, flags = parse_run(out[-1])
        np.testing.assert_allclose(tau[1:], ref[1:], atol=1e-5, err_msg="HOST 开环关节应只有重力前馈")
        self.assertEqual(tau[0], 0.0)

    def test_host_closed_loop_is_pid_plus_gravity(self):
        ref = gravity_ref(Q)
        kp, omax = 0.02, 5.0
        # J2 闭环追 +10 deg 误差；J1 开环；其余开环
        out = self.session(["state 6", q_line(Q), "run", "host 2 50.0 1", "run"], kp=kp, omax=omax)
        tau, cmd, _ = parse_run(out[-1])
        err = 50.0 - Q[2]
        np.testing.assert_allclose(tau[2], kp * err + ref[2], atol=1e-5, err_msg="闭环关节 = PID + 重力")
        for j in (1, 3, 4, 5, 6):
            self.assertAlmostEqual(tau[j], ref[j], places=5, msg=f"开环关节 J{j} 只有重力")

    def test_sum_is_clamped_to_total_max(self):
        """PID 自己先被 output_max 限到 ±omax；和总限幅相同时，代数和仍被总限幅截住"""
        ref = gravity_ref(Q)
        omax = 0.5
        # J2 的重力是负的（ref[2] < 0），所以让 PID 往负方向饱和：目标远小于当前角，PID -> -omax，再叠重力 -> 超出
        out = self.session(["state 6", q_line(Q), "run", "host 2 -170.0 1", "run"], kp=1.0, omax=omax)
        tau, cmd, _ = parse_run(out[-1])
        self.assertLess(ref[2], 0.0)
        self.assertLess(-omax + ref[2], -omax, "构造的前提：不限幅的话 PID+重力会超出 -omax")
        self.assertTrue(np.all(np.abs(tau) <= omax + 1e-6), f"PID+重力不能超过总限幅：{tau}")
        self.assertAlmostEqual(tau[2], -omax, places=5, msg="PID 饱和 + 同号重力，和应被截在 -total_output_max 上")
        np.testing.assert_allclose(cmd, ref, atol=1e-5,
                                   err_msg="重力指令本身保持独立，不按 PID 或总限幅预截断")
        # 反方向：重力与 PID 相反，不会超限，和就是代数和
        out = self.session(["state 6", q_line(Q), "run", "host 2 170.0 1", "run"], kp=1.0, omax=omax)
        tau, _, _ = parse_run(out[-1])
        self.assertAlmostEqual(tau[2], omax + ref[2], places=5, msg="反向时不触发限幅，输出 = 饱和 PID + 重力")

    def test_pid_and_gravity_limits_are_independent(self):
        """PID 被 output_max 截住后，重力仍可独立叠加；最终和只要没到总限幅就不会被 PID 限幅二次截断"""
        ref = gravity_ref(Q)
        pid_max = 0.05
        total_max = 0.3
        out = self.session(["state 6", q_line(Q), "run", "host 2 -170.0 1", "run"],
                           kp=1.0, omax=pid_max, tmax=total_max)
        tau, _, _ = parse_run(out[-1])
        self.assertLess(ref[2], 0.0)
        self.assertAlmostEqual(tau[2], -pid_max + ref[2], places=5,
                               msg="闭环输出应为饱和 PID 与重力的代数和")
        self.assertGreater(abs(tau[2]), pid_max + 1e-6,
                           "重力不能被 PID 的 output_max 截断")
        self.assertLessEqual(abs(tau[2]), total_max + 1e-6)

    def test_total_limit_still_applies_without_gravity(self):
        """重力掩码全关时，PID 仍必须经过总力矩限幅"""
        out = self.session(["state 6", q_line(Q), "run", "host 2 -170.0 1", "run"],
                           mask=0, kp=1.0, omax=5.0, tmax=0.3)
        tau, _, _ = parse_run(out[-1])
        self.assertAlmostEqual(tau[2], -0.3, places=5,
                               msg="没有重力时，PID 输出也要被总限幅截住")

    def test_total_limit_clamps_sum_not_gravity_before_pid(self):
        """总限幅只能在 PID + 重力之后做；不能先把重力截到总限幅，再与反向 PID 抵消"""
        ref = gravity_ref(Q)
        limit = 0.05
        out = self.session(["state 6", q_line(Q), "run", "host 2 170.0 1", "run"],
                           kp=1.0, omax=limit, tmax=limit)
        tau, cmd, _ = parse_run(out[-1])
        self.assertLess(ref[2], 0.0)
        self.assertAlmostEqual(cmd[2], ref[2], places=5,
                               msg="重力前馈本身不应被总限幅预先截断")
        self.assertAlmostEqual(tau[2], -limit, places=5,
                               msg="应为 PID + 重力之和再被总限幅截住，而不是被反向 PID 抵消成 0")

    def test_mask_disables_individual_joints_in_host(self):
        ref = gravity_ref(Q)
        mask = 0b0001010                                  # 只开 J1、J3
        out = self.session(["state 6", q_line(Q), "run", "run"], mask=mask, omax=5.0)
        tau, cmd, _ = parse_run(out[-1])
        for j in range(7):
            if (mask >> j) & 1:
                self.assertAlmostEqual(tau[j], ref[j], places=5)
            else:
                self.assertEqual(tau[j], 0.0, f"J{j} 被掩码关掉，必须零力矩")
        self.assertTrue(np.any(cmd == 0.0))

    def test_gravity_disabled_gives_zero_in_host_open_loop(self):
        out = self.session(["state 6", q_line(Q), "run", "run"], en=0)
        tau, cmd, flags = parse_run(out[-1])
        self.assertTrue(np.all(tau == 0.0), "总开关关了，开环关节必须零力矩")
        self.assertEqual(flags[0], 0)
        self.assertTrue(np.all(cmd == 0.0))

    def test_host_to_protected_leaves_no_gravity(self):
        out = self.session(["state 6", q_line(Q), "run", "run", "state 0", "run"], omax=5.0)
        tau_h, _, _ = parse_run(out[-3])
        tau_p, _, _ = parse_run(out[-1])
        self.assertTrue(np.any(tau_h != 0.0))
        self.assertTrue(np.all(tau_p == 0.0), "退出 HOST 进 PROTECTED 后必须零指令，不残留重力")

    def test_stop_host_control_keeps_gravity_hold_but_no_pid(self):
        """Arm_StopHostControl 之后仍在 HOST：PID 清零，但下个周期起重力前馈照常（新语义，会在报告里提示）"""
        ref = gravity_ref(Q)
        out = self.session(["state 6", q_line(Q), "run", "host 2 50.0 1", "run", "stop", "run"], kp=0.02, omax=5.0)
        tau, _, _ = parse_run(out[-1])
        np.testing.assert_allclose(tau[1:], ref[1:], atol=1e-5)

    def test_nonfinite_angle_gives_zero_not_nan(self):
        bad = list(Q)
        bad[3] = float("nan")
        out = self.session(["state 6", q_line(bad).replace("nan", "nan"), "run"], omax=5.0)
        tau, cmd, flags = parse_run(out[-1])
        self.assertTrue(np.all(np.isfinite(tau)), f"NaN 不能漏到电机指令：{tau}")
        self.assertEqual(flags[0], 0)

    def test_compile_switch_off_restores_zero_torque_for_open_loop(self):
        if not self.have_off:
            self.skipTest("ARM_GRAVITY_OUTPUT=0 版本没编出来")
        s = Sim(self.exe_off, omax=5.0)
        s.add("state 6", q_line(Q), "run", "run")
        out = s.run(self.exe_off)
        tau, cmd, flags = parse_run(out[-1])
        self.assertTrue(np.all(tau == 0.0), "编译开关为 0：HOST 开环关节回到零力矩，与原行为一致")
        self.assertTrue(np.any(cmd != 0.0), "但重力仍在计算")
        self.assertEqual(flags[1], 0)

    def test_sign_minus_one_flips_host_feedforward(self):
        ref = gravity_ref(Q)
        out = self.session(["state 6", q_line(Q), "run", "run"], sign=-1, omax=5.0)
        tau, _, _ = parse_run(out[-1])
        np.testing.assert_allclose(tau[1:], -ref[1:], atol=1e-5)


if __name__ == "__main__":
    unittest.main()
