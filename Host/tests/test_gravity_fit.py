"""tools/gravity_fit.py 测试：回归矩阵对势能数值梯度、秩、合成数据恢复、CSV 读取与命令行。"""

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

HOST = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST))
sys.path.insert(0, str(HOST / "tools"))

import numpy as np  # noqa: E402

import gravity_fit as gf  # noqa: E402
from armhost.kinematics import DHTable  # noqa: E402
from armhost.model.recorder import header  # noqa: E402

TABLE = DHTable()


def _write_csv(path: Path, q_deg, tau, vel=None, state=6, n=7):
    cols = header(n)
    with open(path, "w", newline="", encoding="utf-8") as f:
        f.write(",".join(cols) + "\n")
        for k in range(len(q_deg)):
            r = {c: 0.0 for c in cols}
            r["arm_state"] = state if np.isscalar(state) else state[k]
            for j in range(n):
                r[f"J{j}_angle_deg"] = q_deg[k][j]
                r[f"J{j}_torque_nm"] = tau[k][j]
                r[f"J{j}_torque_cmd_nm"] = tau[k][j]
                r[f"J{j}_velocity_dps"] = 0.0 if vel is None else vel[k][j]
            f.write(",".join(str(r[c]) for c in cols) + "\n")


def _poses(rng, n_pose, spread=120.0):
    return rng.uniform(-spread, spread, size=(n_pose, 7))


class TestRegressor(unittest.TestCase):
    def test_matches_potential_gradient(self):
        rng = np.random.default_rng(1)
        for _ in range(10):
            q = rng.uniform(-np.pi, np.pi, 7)
            pi = rng.normal(size=28)
            tau = gf.gravity_regressor(TABLE, q) @ pi
            num = np.zeros(7)
            for j in range(7):
                e = np.zeros(7)
                e[j] = 1e-6
                num[j] = (gf.potential_energy(TABLE, q + e, pi) - gf.potential_energy(TABLE, q - e, pi)) / 2e-6
            np.testing.assert_allclose(tau, num, atol=1e-6)

    def test_structure(self):
        q = np.radians([10, 20, 30, 40, 50, 60, 70])
        Y = gf.gravity_regressor(TABLE, q)
        self.assertEqual(Y.shape, (7, 28))
        self.assertTrue(np.all(Y[0] == 0), "J1 轴竖直，第一行应恒为 0")
        self.assertTrue(np.all(Y[:, :4] == 0), "link1 的参数不进入 Y")
        for j in range(7):                       # 上三角：j 行只含 i>=j 的连杆
            self.assertTrue(np.all(Y[j, :4 * j] == 0))

    def test_numerical_rank_is_12(self):
        rng = np.random.default_rng(2)
        Y = np.vstack([gf.gravity_regressor(TABLE, np.radians(q)) for q in _poses(rng, 80, 180)])
        sv = np.linalg.svd(Y, compute_uv=False)
        self.assertEqual(int(np.sum(sv > sv[0] * 1e-9)), 12)


class TestFit(unittest.TestCase):
    def test_recovers_synthetic_torque(self):
        rng = np.random.default_rng(3)
        pi = rng.normal(scale=0.05, size=28)
        with tempfile.TemporaryDirectory() as d:
            paths = []
            for k in range(3):
                q = _poses(rng, 25, 170)
                tau = [gf.gravity_regressor(TABLE, np.radians(r)) @ pi for r in q]
                p = Path(d) / f"s{k}.csv"
                _write_csv(p, q, tau)
                paths.append(str(p))
            out = Path(d) / "fit.json"
            with contextlib.redirect_stdout(io.StringIO()):
                rc = gf.main([*paths, "--out", str(out)])
            self.assertEqual(rc, 0)
            res = json.loads(out.read_text(encoding="utf-8"))
        self.assertEqual(res["rank"], 12)
        self.assertLess(max(s["rmse"] for s in res["stats"]), 1e-9)
        self.assertEqual(len(res["cv"]), 3)
        self.assertLess(max(c["rmse"] for c in res["cv"]), 1e-8, "无噪声数据，留一预测也应精确")

    def test_bias_and_noise(self):
        rng = np.random.default_rng(4)
        pi = rng.normal(scale=0.05, size=28)
        bias = rng.normal(scale=0.02, size=7)
        q = _poses(rng, 80, 170)
        tau = np.array([gf.gravity_regressor(TABLE, np.radians(r)) @ pi + bias for r in q])
        tau += rng.normal(scale=1e-3, size=tau.shape)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "b.csv"
            _write_csv(p, q, tau)
            ds = gf.load_dataset(p)
        Y, t, _ = gf.build_system(TABLE, [ds], True, gf.G)
        x, rank, _ = gf.solve(Y, t, 1e-6)
        self.assertEqual(rank, 12 + 7, "每个零偏只出现在自己那一行，7 个都可辨识")
        np.testing.assert_allclose(x[28:], bias, atol=5e-3)

    def test_sign_flips_pi_only(self):
        rng = np.random.default_rng(5)
        pi = rng.normal(scale=0.05, size=28)
        q = _poses(rng, 30, 170)
        tau = np.array([gf.gravity_regressor(TABLE, np.radians(r)) @ pi for r in q])
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "x.csv"
            _write_csv(p, q, tau)
            ds = gf.load_dataset(p)
        Y, t1, _ = gf.build_system(TABLE, [ds], False, gf.G, 1.0)
        _, t2, _ = gf.build_system(TABLE, [ds], False, gf.G, -1.0)
        x1, _, _ = gf.solve(Y, t1, 1e-6)
        x2, _, _ = gf.solve(Y, t2, 1e-6)
        np.testing.assert_allclose(x1, -x2, atol=1e-9)


class TestLoad(unittest.TestCase):
    def test_filters(self):
        rng = np.random.default_rng(6)
        q = _poses(rng, 6)
        vel = np.zeros((6, 7))
        vel[1, 4] = 80
        vel[4, 0] = -30
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "f.csv"
            _write_csv(p, q, np.zeros((6, 7)), vel, state=[6, 6, 5, 6, 6, 5])
            ds = gf.load_dataset(p)
            self.assertEqual(len(ds.q_deg), 6)
            self.assertEqual(len(gf.filter_dataset(ds, [6], None).q_deg), 4)
            f = gf.filter_dataset(ds, None, 20.0)
            self.assertEqual(len(f.q_deg), 4)
            self.assertEqual(f.skipped["moving"], 2)
            self.assertEqual(len(gf.filter_dataset(ds, [6], 20.0).q_deg), 2)

    def test_bad_files(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.csv"
            p.write_text("a,b\n1,2\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                gf.load_dataset(p)
            q = np.zeros((1, 7))
            _write_csv(Path(d) / "ok.csv", q, q)
            with self.assertRaises(ValueError):
                gf.load_dataset(Path(d) / "ok.csv", n_expect=6)
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(gf.main([str(p)]), 2)
                self.assertEqual(gf.main([str(Path(d) / "nope.csv")]), 2)

    def test_real_logs_if_present(self):
        files = sorted((HOST / "logs" / "gravity").glob("*.csv"))
        if not files:
            self.skipTest("没有真实采集数据")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            self.assertEqual(gf.main([str(f) for f in files]), 0)
        self.assertIn("秩 12 / 28", buf.getvalue())


if __name__ == "__main__":
    unittest.main()
