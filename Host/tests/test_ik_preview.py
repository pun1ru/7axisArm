"""Host 侧 IK 预览：与 ArmSim 解析解一致，并能算出受关节限位约束的 ψ 区间。"""

import unittest

from armhost.kinematics import (DHTable, DHRow, IkStatus, compute_arm_angle, compute_branch,
                                euler_zyx_deg, feasible_psi_intervals, forward_frames,
                                is_psi_feasible, make_srs_param, solve_ik,
                                transform_from_xyz_rpy)


class TestIkPreview(unittest.TestCase):
    def setUp(self):
        self.dh = DHTable([
            DHRow(0.0, 0.4105, -0.0897, -72.5122038),
            DHRow(90.0, 0.0, 0.0, -90.0),
            DHRow(90.0, 0.0, 0.188, -90.0),
            DHRow(90.0, 0.0, -0.0025, 170.7537190),
            DHRow(90.0, 0.0, 0.207, 10.6949890),
            DHRow(90.0, 0.0, 0.0, -148.8585943),
            DHRow(90.0, 0.0, 0.0, 0.0),
        ], "test")
        self.param = make_srs_param(self.dh)
        self.assertIsNotNone(self.param)
        self.q = [10.0, -30.0, 20.0, 40.0, 15.0, -25.0, 30.0]
        self.frames = forward_frames(self.dh, self.q)
        self.target = self.frames[-1]
        self.psi = compute_arm_angle(self.param, self.frames)
        self.branch = compute_branch(self.dh, self.q)

    def test_solve_ik_round_trip_matches_firmware_convention(self):
        result = solve_ik(self.param, self.target, self.branch, self.psi)
        self.assertEqual(result.status, IkStatus.OK)
        for actual, expected in zip(result.q_deg, self.q):
            self.assertAlmostEqual((actual - expected + 180.0) % 360.0 - 180.0, 0.0, places=6)
        self.assertAlmostEqual(compute_arm_angle(self.param, forward_frames(self.dh, result.q_deg)),
                               self.psi, places=6)

    def test_pose_transform_round_trip(self):
        pose = self.target[:3, 3]
        rpy = euler_zyx_deg(self.target[:3, :3])
        rebuilt = transform_from_xyz_rpy(*pose, *rpy)
        for actual, expected in zip(rebuilt[:3, 3], pose):
            self.assertAlmostEqual(actual, expected, places=9)
        for actual, expected in zip(rebuilt[:3, :3].ravel(), self.target[:3, :3].ravel()):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_feasible_psi_respects_joint_limits(self):
        limits = tuple((q - 5.0, q + 5.0) for q in self.q)
        intervals = feasible_psi_intervals(self.param, self.target, self.branch,
                                           limits, self.q, step_deg=1.0)
        self.assertTrue(intervals)
        self.assertTrue(any(lo <= self.psi <= hi for lo, hi in intervals))
        self.assertGreater(intervals[0][0], -179.9)
        self.assertLess(intervals[-1][1], 179.9)

    def test_unreachable_pose_has_no_psi_interval(self):
        target = transform_from_xyz_rpy(3.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        limits = tuple((-180.0, 180.0) for _ in range(7))
        self.assertEqual(feasible_psi_intervals(self.param, target, self.branch, limits, self.q), [])
        self.assertEqual(solve_ik(self.param, target, self.branch, 0.0).status, IkStatus.OUT_OF_REACH)

    def test_feasible_psi_uses_requested_joint_limits(self):
        limits = ((-90.0, 90.0), (-20.0, 175.0), (-175.0, 175.0), (-110.0, 110.0),
                  (-175.0, 175.0), (-120.0, 70.0), (-175.0, 175.0))
        q = [10.0, 30.0, 20.0, 40.0, 15.0, -25.0, 30.0]
        frames = forward_frames(self.dh, q)
        target = frames[-1]
        psi = compute_arm_angle(self.param, frames)
        branch = compute_branch(self.dh, q)

        self.assertTrue(is_psi_feasible(self.param, target, branch, psi, limits, q))
        intervals = feasible_psi_intervals(self.param, target, branch, limits, q, step_deg=1.0)
        self.assertTrue(intervals)
        self.assertTrue(any(lo <= psi <= hi for lo, hi in intervals))

    def test_near_singular_shoulder_and_wrist_psi_are_not_feasible(self):
        limits = ((-90.0, 90.0), (-20.0, 175.0), (-175.0, 175.0), (-110.0, 110.0),
                  (-175.0, 175.0), (-120.0, 70.0), (-175.0, 175.0))
        cases = {
            "shoulder": [10.0, 90.5, 20.0, 40.0, 15.0, -25.0, 30.0],
            "wrist": [10.0, 30.0, 20.0, 40.0, 15.0, -31.6414057, 30.0],
        }
        for name, q in cases.items():
            with self.subTest(name=name):
                frames = forward_frames(self.dh, q)
                target = frames[-1]
                psi = compute_arm_angle(self.param, frames)
                branch = compute_branch(self.dh, q)
                solved = solve_ik(self.param, target, branch, psi)
                self.assertEqual(solved.status, IkStatus.OK)
                self.assertFalse(is_psi_feasible(self.param, target, branch, psi, limits, q))


if __name__ == "__main__":
    unittest.main()
