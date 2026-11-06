"""工作空间采样测试。"""

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from armhost.kinematics import (DHTable, branch_indices_deg, compute_branch, forward_frames,  # noqa: E402
                                forward_positions_deg, sample_workspace)
from armhost.kinematics.workspace import sample_joint_configurations  # noqa: E402


class TestWorkspace(unittest.TestCase):
    def test_batch_fk_matches_frame_fk(self):
        table = DHTable()
        q = np.array([[0.0] * 7,
                      [10.0, -20.0, 30.0, -40.0, 50.0, -60.0, 70.0],
                      [-179.0, 179.0, 0.0, 90.0, -90.0, 45.0, -45.0]])
        ref = np.array([forward_frames(table, row)[-1][:3, 3] for row in q])
        np.testing.assert_allclose(forward_positions_deg(table, q), ref, atol=1e-12)

    def test_sampling_is_deterministic_and_voxelized(self):
        table = DHTable()
        limits = ((-90.0, 90.0), (-120.0, 120.0), (-150.0, 150.0), (-170.0, 170.0),
                  (-120.0, 120.0), (-180.0, 180.0), (-180.0, 180.0))
        a = sample_workspace(table, limits, samples=12_000, voxel_m=0.02, seed=7)
        b = sample_workspace(table, limits, samples=12_000, voxel_m=0.02, seed=7)
        np.testing.assert_array_equal(a.points_m, b.points_m)
        np.testing.assert_array_equal(a.density, b.density)
        self.assertEqual(a.samples, 12_000)
        self.assertGreater(len(a.points_m), 0)
        self.assertLessEqual(len(a.points_m), 18_000)
        self.assertEqual(len(a.points_m), len(a.density))
        self.assertTrue(np.isfinite(a.points_m).all())
        self.assertAlmostEqual(float(a.density.max()), 1.0)
        self.assertGreaterEqual(a.voxel_m, 0.02)

    def test_invalid_limits_rejected(self):
        table = DHTable()
        with self.assertRaises(ValueError):
            sample_workspace(table, ((0.0, 0.0),) * 7, samples=100)
        with self.assertRaises(ValueError):
            sample_workspace(table, ((0.0, 1.0),) * 6, samples=100)

    def test_branch_filter_matches_scalar_and_limits_configurations(self):
        table = DHTable()
        limits = ((-180.0, 180.0),) * 7
        q = np.random.default_rng(3).uniform(-180.0, 180.0, (200, 7))
        idx = branch_indices_deg(table, q)
        self.assertEqual([int(v) for v in idx], [compute_branch(table, row) for row in q])

        for branch in range(8):
            qb = sample_joint_configurations(table, limits, samples=2_000, seed=11, branch=branch)
            self.assertEqual(len(qb), 2_000)
            self.assertTrue(np.all(branch_indices_deg(table, qb) == branch))

    def test_branch_specific_cloud_keeps_branch_metadata(self):
        table = DHTable()
        cloud = sample_workspace(table, ((-180.0, 180.0),) * 7, samples=4_000,
                                 voxel_m=0.03, seed=5, branch=5)
        self.assertEqual(cloud.branch, 5)
        self.assertEqual(cloud.samples, 4_000)
        self.assertGreater(len(cloud.points_m), 0)


if __name__ == "__main__":
    unittest.main()
