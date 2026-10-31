"""固件 arm_dynamics.c 与上位机 gravity_regressor() 的数值对拍。

把固件里同一份 arm_dynamics.c / dh.c 用主机 C 编译器（MSVC cl 或 gcc/clang，找不到就跳过）编成可执行文件，
喂随机位形，对比 float32 的 C 结果和 float64 的 Python Y(q) @ pi，验证：
  1. q=0 时 J0 重力矩严格为 0；
  2. 28 维 pi 版本、12 维 beta 版本与 Python 参考的最大误差；
  3. 12 列 SEL 与固件宏一致，B @ beta 重建 Y @ pi 到机器精度；
  4. 使能关闭 / 参数表为空 / q 含 NaN 或 Inf 时输出清零且返回 false。
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

from armhost.kinematics import BUILTIN_DH, DHTable  # noqa: E402
from armhost.model.gravity_fit import G, gravity_regressor  # noqa: E402

FW = ROOT.parent / "Real" / "DIYcontroller"
ARM = FW / "LibSE" / "Module" / "Arm"
KIN = FW / "LibSE" / "Algorithm" / "Kinematics"
MATH = FW / "LibSE" / "Algorithm" / "Math"
DRIVER = ROOT / "tests" / "c_gravity_driver.c"

# 用户给的实测 pi（link0..link6，每行 hx hy hz m）
PI_USER = np.array([
    +0.00000, +0.00000, -0.00000, +0.00000,
    -0.00052, +0.08218, -0.00000, +0.00000,
    -0.00375, +0.00012, -0.08218, +0.01216,
    +0.00040, -0.01262, -0.00012, +0.01216,
    +0.00053, +0.00012, +0.01262, +0.01342,
    +0.00137, +0.00458, -0.00012, +0.01342,
    -0.00116, +0.00064, -0.00458, +0.01342])
SEL = [4, 5, 8, 9, 12, 13, 16, 17, 20, 21, 24, 25]
LIMITS = np.array([[-90, 90], [-95, 100], [-180, 180], [-120, 0], [-180, 180], [-60, 90], [-180, 180]], float)


def _find_vcvars():
    for edition in ("Community", "Professional", "Enterprise", "BuildTools"):
        for base in (r"C:\Program Files\Microsoft Visual Studio\2022", r"C:\Program Files (x86)\Microsoft Visual Studio\2022"):
            p = Path(base) / edition / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
            if p.exists():
                return p
    return None


def build_driver(out: Path) -> bool:
    srcs = [DRIVER, ARM / "arm_dynamics.c", KIN / "dh.c"]
    incs = [ARM, KIN, MATH]
    for cc in ("gcc", "clang"):
        exe = shutil.which(cc)
        if exe:
            cmd = [exe, "-std=c99", "-O2", "-Wall", "-Wextra", *[f"-I{i}" for i in incs], *map(str, srcs), "-lm", "-o", str(out)]
            return subprocess.run(cmd, capture_output=True).returncode == 0
    vc = _find_vcvars()
    if vc is None:
        return False
    flags = " ".join(f'/I"{i}"' for i in incs)
    files = " ".join(f'"{s}"' for s in srcs)
    bat = f'call "{vc}" >nul && cl /nologo /O2 /W3 {flags} {files} /Fe:"{out}" /Fo:"{out.parent}\\\\"'
    r = subprocess.run(bat, shell=True, capture_output=True, text=True, encoding="utf-8", errors="replace",
                       cwd=out.parent)
    if r.returncode != 0:
        print(r.stdout, r.stderr)
    return r.returncode == 0


def dh_text(table=None) -> str:
    rows = (table.rows if table else DHTable().rows)
    return "\n".join(" ".join(map(_num, (r.alpha_deg, r.a_m, r.d_m, r.theta_offset_deg))) for r in rows)


def _num(v) -> str:
    """float() 之后再格式化：numpy 2.x 的 repr 会变成 'np.float64(..)'，C 的 scanf 读不了；nan/inf 用 C 认识的写法"""
    return f"{float(v):.17g}"


def run_driver(exe: Path, qs, pi, beta, g=G, mode=None, extra=None, table=None) -> np.ndarray:
    head = (f"{len(qs)}\n" + " ".join(map(_num, pi)) + "\n" + " ".join(map(_num, beta)) + f"\n{_num(g)}\n"
            + dh_text(table) + "\n")
    body = []
    for k, q in enumerate(qs):
        line = " ".join(map(_num, q))
        if mode == "model":
            line += " " + " ".join(_num(x) for x in extra[k])
        body.append(line)
    r = subprocess.run([str(exe)] + ([mode] if mode else []), input=head + "\n".join(body) + "\n",
                       capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=60)
    assert r.returncode == 0, r.stderr
    return np.array([[float(x) for x in ln.split()] for ln in r.stdout.strip().splitlines()])


def beta_from_pi(pi, rng) -> np.ndarray:
    qs = rng.uniform(LIMITS[:, 0], LIMITS[:, 1], size=(300, 7))
    Y = np.vstack([gravity_regressor(DHTable(), np.radians(q)) for q in qs])
    return np.linalg.lstsq(Y[:, SEL], Y @ pi, rcond=None)[0]


class TestFirmwareGravity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.exe = Path(cls.tmp.name) / ("gdrv.exe" if os.name == "nt" else "gdrv")
        if not (ARM / "arm_dynamics.c").read_text(encoding="utf-8").strip():
            raise unittest.SkipTest("arm_dynamics.c 是空文件")
        if not build_driver(cls.exe):
            raise unittest.SkipTest("没有可用的主机 C 编译器（gcc / clang / MSVC）")
        cls.rng = np.random.default_rng(2026)
        cls.beta = beta_from_pi(PI_USER, np.random.default_rng(1))

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_sel_matches_firmware_macro(self):
        h = (ARM / "arm_dynamics.h").read_text(encoding="utf-8")
        m = re.search(r"#define\s+ARM_GRAVITY_BETA_SEL\s*\{([^}]*)\}", h)
        self.assertIsNotNone(m)
        self.assertEqual([int(x.strip().rstrip("U")) for x in m.group(1).split(",")], SEL)
        self.assertIn("ARM_GRAVITY_DEFAULT_G   9.81f", re.sub(r"\s+", " ", h).replace("ARM_GRAVITY_DEFAULT_G 9.81f", "ARM_GRAVITY_DEFAULT_G   9.81f"))

    def test_g_default_matches_fit(self):
        self.assertEqual(G, 9.81)

    def test_q_zero_joint0_is_zero(self):
        out = run_driver(self.exe, [[0.0] * 7], PI_USER, self.beta)[0]
        print("\n  q=0  pi版:  ", np.array2string(out[:7], precision=6, suppress_small=False))
        print("  q=0  beta版:", np.array2string(out[7:], precision=6, suppress_small=False))
        self.assertEqual(out[0], 0.0, "J0 轴竖直，重力矩必须严格为 0")
        self.assertEqual(out[7], 0.0)
        ref = gravity_regressor(DHTable(), np.zeros(7)) @ PI_USER
        np.testing.assert_allclose(out[:7], ref, atol=1e-5)

    def test_random_poses_match_python_reference(self):
        qs = self.rng.uniform(LIMITS[:, 0], LIMITS[:, 1], size=(2000, 7))
        out = run_driver(self.exe, qs, PI_USER, self.beta)
        ref = np.array([gravity_regressor(DHTable(), np.radians(q)) @ PI_USER for q in qs])
        e_pi = np.abs(out[:, :7] - ref).max()
        e_beta = np.abs(out[:, 7:] - ref).max()
        e_pb = np.abs(out[:, :7] - out[:, 7:]).max()
        print(f"\n  2000 个随机位形：max|C_pi - Py| = {e_pi:.3e}   max|C_beta - Py| = {e_beta:.3e}"
              f"   max|C_pi - C_beta| = {e_pb:.3e} N*m   (最大力矩 {np.abs(ref).max():.3f})")
        self.assertLess(e_pi, 1e-4)
        self.assertLess(e_beta, 1e-4)
        self.assertTrue(np.all(out[:, 0] == 0.0), "J0 全程必须为 0")
        self.assertTrue(np.all(out[:, 7] == 0.0))

    def test_random_nonzero_pi_all_joints(self):
        """pi 取随机量级与用户数据相当的全 28 维值（含 link0、hz、m），覆盖所有列"""
        r = np.random.default_rng(7)
        pi = r.normal(scale=0.05, size=28)
        beta = beta_from_pi(pi, r)
        qs = r.uniform(-180, 180, size=(1500, 7))                # 含限位外，覆盖全角度范围
        out = run_driver(self.exe, qs, pi, beta)
        ref = np.array([gravity_regressor(DHTable(), np.radians(q)) @ pi for q in qs])
        e_pi, e_beta = np.abs(out[:, :7] - ref).max(), np.abs(out[:, 7:] - ref).max()
        print(f"\n  全 28 维随机 pi：max|C_pi - Py| = {e_pi:.3e}   max|C_beta - Py| = {e_beta:.3e} N*m")
        self.assertLess(e_pi, 1e-4)
        self.assertLess(e_beta, 1e-4)

    def test_beta_reconstructs_Y_pi_to_machine_precision(self):
        qs = self.rng.uniform(LIMITS[:, 0], LIMITS[:, 1], size=(400, 7))
        Y = np.vstack([gravity_regressor(DHTable(), np.radians(q)) for q in qs])
        B = Y[:, SEL]
        beta_ref = np.linalg.lstsq(B, Y @ PI_USER, rcond=None)[0]
        err = np.abs(B @ beta_ref - Y @ PI_USER).max()
        print(f"\n  rank(B)={np.linalg.matrix_rank(B)}  cond(B)={np.linalg.cond(B):.3f}  max|B beta - Y pi| = {err:.3e}")
        self.assertEqual(np.linalg.matrix_rank(B), 12)
        self.assertLess(err, 1e-12)
        np.testing.assert_allclose(beta_ref, self.beta, atol=1e-12)

    def test_model_wrapper_guards(self):
        q_ok = [10.0, -30.0, 40.0, -50.0, 20.0, 10.0, 5.0]
        q_nan = list(q_ok)
        q_nan[3] = float("nan")
        q_inf = list(q_ok)
        q_inf[1] = float("inf")
        # 每行： enable use_beta pi_null beta_null g
        rows = [(q_ok, (1, 1, 0, 0, 9.81)), (q_ok, (1, 0, 0, 0, 9.81)), (q_ok, (0, 1, 0, 0, 9.81)),
                (q_ok, (1, 1, 0, 1, 9.81)), (q_ok, (1, 0, 1, 0, 9.81)), (q_nan, (1, 1, 0, 0, 9.81)),
                (q_inf, (1, 0, 0, 0, 9.81)), (q_ok, (1, 1, 0, 0, 0.0))]
        out = run_driver(self.exe, [r[0] for r in rows], PI_USER, self.beta, mode="model", extra=[r[1] for r in rows])
        ref = gravity_regressor(DHTable(), np.radians(q_ok)) @ PI_USER
        np.testing.assert_allclose(out[0, 1:], ref, atol=1e-5)
        self.assertEqual(out[0, 0], 1)
        np.testing.assert_allclose(out[1, 1:], ref, atol=1e-5)
        np.testing.assert_allclose(out[7, 1:], ref, atol=1e-5)              # g=0 -> 默认 9.81
        for k in (2, 3, 4, 5, 6):
            self.assertEqual(out[k, 0], 0, f"第 {k} 行应返回 false")
            self.assertTrue(np.all(out[k, 1:] == 0.0), f"第 {k} 行失败路径必须把 tau 清零：{out[k]}")

    def test_command_sign_clamp_and_guards(self):
        """ArmDynamics_GravityCommand：符号只乘一次、按传入限幅截断、无效 / 限幅没填 / NaN 一律输出 0"""
        lim = [0.2, 2.5, 0.5, 0.6, 0.2, 0.1, 0.05]            # module_init.c 里各关节的总力矩限幅
        model = [0.0, -3.0, 0.3, 0.7, -0.1, 0.15, 0.01]
        nan, inf = float("nan"), float("inf")
        ALL = 0x7F
        rows = [(model, 1, 1, ALL, lim), (model, 1, -1, ALL, lim), (model, 1, 0, ALL, lim),    # 同号 / 反号 / 未配按 +1
                (model, 0, 1, ALL, lim),                                                         # 模型无效
                (model, 1, 1, ALL, [0.0, 2.5, -1.0, nan, 0.2, 0.1, 0.05]),                       # 限幅 0 / 负 / NaN
                ([nan, inf, -inf, 0.3, nan, 0.15, 0.01], 1, 1, ALL, lim),                        # 模型值非有限
                (model, 1, 1, 0b0001010, lim),                                                   # 只开 J1、J3
                (model, 1, -1, 0b1000000, lim),                                                  # 只开 J6，反号
                (model, 1, 1, 0x00, lim)]                                                        # 全关
        qs = [m + [float(v), float(s), float(k)] + l for m, v, s, k, l in rows]
        out = run_driver(self.exe, qs, PI_USER, self.beta, mode="cmd")
        exp0 = [0.0, -2.5, 0.3, 0.6, -0.1, 0.1, 0.01]          # 限幅后：J1 -3.0 -> -2.5，J3 0.7 -> 0.6，J5 0.15 -> 0.1
        np.testing.assert_allclose(out[0], exp0, atol=1e-7)
        np.testing.assert_allclose(out[1], [-x for x in exp0], atol=1e-7, err_msg="sign=-1 应整体取反且仍限幅")
        np.testing.assert_allclose(out[2], exp0, atol=1e-7, err_msg="sign=0 按 +1")
        self.assertTrue(np.all(out[3] == 0.0), "valid=false 必须全 0")
        np.testing.assert_allclose(out[4], [0.0, -2.5, 0.0, 0.0, -0.1, 0.1, 0.01], atol=1e-7)
        np.testing.assert_allclose(out[5], [0.0, 0.0, 0.0, 0.3, 0.0, 0.1, 0.01], atol=1e-7)
        np.testing.assert_allclose(out[6], [0.0, -2.5, 0.0, 0.6, 0.0, 0.0, 0.0], atol=1e-7,
                                   err_msg="掩码只开 J1、J3：其余轴必须是 0，开着的轴与全开时一致")
        np.testing.assert_allclose(out[7], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01], atol=1e-7)
        self.assertTrue(np.all(out[8] == 0.0), "掩码全 0 必须没有任何前馈")
        self.assertTrue(np.all(np.abs(out) <= np.array(lim) + 1e-7), "任何情况都不能超过传入限幅")
        self.assertTrue(np.all(np.isfinite(out)))

    def test_module_init_mask_macros(self):
        """module_init.c 的 ARM_GRAVITY_EN_Jx 合成的掩码必须是 7 位、与结构体字段对得上"""
        src = (FW / "App" / "module_init.c").read_text(encoding="utf-8")
        en = [int(re.search(rf"#define\s+ARM_GRAVITY_EN_J{i}\s+(\d)U", src).group(1)) for i in range(7)]
        self.assertTrue(all(v in (0, 1) for v in en))
        self.assertIn(".joint_mask = ARM_GRAVITY_JOINT_MASK", src)
        for i in range(7):
            self.assertRegex(src, rf"ARM_GRAVITY_EN_J{i}\s*<<\s*{i}\)")

    def test_sign_convention_is_single_flip(self):
        """符号只乘一次：g 取负 == 整体取负，和 pi / beta 取负等价"""
        qs = self.rng.uniform(LIMITS[:, 0], LIMITS[:, 1], size=(50, 7))
        a = run_driver(self.exe, qs, PI_USER, self.beta, g=G)
        b = run_driver(self.exe, qs, -PI_USER, -self.beta, g=G)
        np.testing.assert_allclose(a, -b, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
