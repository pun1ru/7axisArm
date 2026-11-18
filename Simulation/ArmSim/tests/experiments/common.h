#pragma once
/* 解析解 vs 数值解 离线对比：公共部分。解析解的调用方式照搬固件 SelectAutoPsi（ψ 每 2° 采样、
 * 肩/腕非奇异、关节限位、按到参考构型的加权位移选最小）。 */
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "ik.h"
#include "ik_numeric.h"
#include "kinematics.h"

inline const DhTable kDh = {{
    {0.0f, 0.4105f, -0.0897f, -72.5122038f},
    {90.0f, 0.0f, 0.0f, -90.0f},
    {90.0f, 0.0f, 0.188f, -90.0f},
    {90.0f, 0.0f, -0.0025f, 170.7537190f},
    {90.0f, 0.0f, 0.207f, 10.6949890f},
    {90.0f, 0.0f, 0.0f, -148.8585943f},
    {90.0f, 0.0f, 0.0f, 0.0f},
}};
inline const JointArray kMin = {-90, -20, -175, -110, -175, -120, -175};
inline const JointArray kMax = {90, 175, 175, 110, 175, 70, 175};

inline const SrsParam &Srs()
{
    static SrsParam p;
    static bool ok = MakeSrsParam(kDh, p);
    if (!ok) { std::fprintf(stderr, "DH is not SRS\n"); std::exit(1); }
    return p;
}

inline Mat4 Fk(const JointArray &q) { return ForwardKinematics(kDh, q); }

inline JointArray RandomQ(std::mt19937 &g)
{
    JointArray q{};
    for (int i = 0; i < kJointCount; ++i)
        q[i] = std::uniform_real_distribution<float>(kMin[i], kMax[i])(g);
    return q;
}

inline double Norm3(const std::array<double, 6> &e, int o)
{
    return std::sqrt(e[o] * e[o] + e[o + 1] * e[o + 1] + e[o + 2] * e[o + 2]);
}
inline double PosErr(const Mat4 &a, const Mat4 &b) { return Norm3(ComputePoseError(a, b), 0); }
inline double RotErrDeg(const Mat4 &a, const Mat4 &b) { return Norm3(ComputePoseError(a, b), 3) * 180.0 / 3.14159265358979; }

inline Branch BranchOf(int index)
{
    Branch b;
    b.valid = true;
    b.shoulder = (index & 1) ? -1 : 1;
    b.elbow = (index & 2) ? -1 : 1;
    b.wrist = (index & 4) ? -1 : 1;
    return b;
}

/* 解析解，一个分支：ψ 采样 + 限位 + 非奇异，取到 ref 的位移平方和最小 */
inline bool AnalyticBranch(const Mat4 &target, const Branch &br, const JointArray &ref, JointArray &out)
{
    const SrsParam &s = Srs();
    bool found = false;
    double best = 1e30;

    for (int k = 0; k <= 180; ++k)
    {
        const IkResult r = SolveIk(s, target, br, -180.0f + 2.0f * static_cast<float>(k));
        if (r.status != IkStatus::Ok || r.shoulder_singular || r.wrist_singular) continue;
        bool good = true;
        for (int j : {1, 5})
            if (std::fabs(std::sin(r.q_deg[j] * kDegToRad + s.theta_offset_rad[j])) < 0.02) good = false;
        JointArray q{};
        double cost = 0.0;
        for (int i = 0; good && i < kJointCount; ++i)
        {
            bool have = false;
            for (int t = -1; t <= 1; ++t)
            {
                const float c = r.q_deg[i] + 360.0f * static_cast<float>(t);
                if (c < kMin[i] || c > kMax[i]) continue;
                if (!have || std::fabs(c - ref[i]) < std::fabs(q[i] - ref[i])) { q[i] = c; have = true; }
            }
            if (!have) good = false;
            else cost += (q[i] - ref[i]) * (q[i] - ref[i]);
        }
        if (good && cost < best) { best = cost; out = q; found = true; }
    }
    return found;
}

inline bool AnalyticAny(const Mat4 &target, const JointArray &ref, JointArray &out)
{
    for (int b = 0; b < 8; ++b)
        if (AnalyticBranch(target, BranchOf(b), ref, out)) return true;
    return false;
}

struct NumRun {
    JointArray q{};
    double pos = 0, rot_deg = 0;
    int cycles = 0;
    bool converged = false, limit_blocked = false;
};

/* 数值解：从 start 出发每周期一步，直到收敛或达到 cap 个周期 */
inline NumRun RunNumeric(const JointArray &start, const Mat4 &target, const IkCostWeights &w,
                         const NumericIkParam &p, int cap)
{
    NumRun r;
    JointArray q = start;

    for (int c = 0; c < cap; ++c)
    {
        const NumericIkStep s = SolveNumericIkStep(kDh, q, target, start, w, kMin, kMax, p);
        r.cycles = c;
        if (!s.ok) break;
        r.limit_blocked = s.limit_mask != 0;
        if (s.converged) { r.converged = true; break; }
        for (int i = 0; i < kJointCount; ++i) q[i] += s.dq_deg[i];
    }
    r.q = q;
    r.pos = PosErr(Fk(q), target);
    r.rot_deg = RotErrDeg(Fk(q), target);
    return r;
}

inline double Pct(std::vector<double> v, double p)
{
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[static_cast<size_t>(p * static_cast<double>(v.size() - 1))];
}
