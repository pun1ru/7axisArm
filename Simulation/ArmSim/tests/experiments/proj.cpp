/* 直接测零空间步的投影质量：e=0，步长/限位放到极大（不被截断），只剩零空间项。
 * 泄漏 = ‖J·dq‖（dq 的一阶任务空间效应，应为 0）；放大 = ‖dq‖_W / ‖z‖_W（W 加权投影应 ≤ 1）。 */
#include "common.h"

static void Sweep(const char *name, const std::vector<std::pair<int, float>> &fix, std::mt19937 &g)
{
    NumericIkParam p;
    p.nullspace_gain = 1.0;
    p.max_step_deg = 1e6; p.max_pos_err_m = 1e6; p.max_rot_err_rad = 1e6;
    JointArray lo, hi;
    lo.fill(-1e6f); hi.fill(1e6f);
    IkCostWeights w{};
    w.weight = {1.0f, 2.0f, 0.5f, 3.0f, 1.0f, 1.0f, 2.0f};   // 非均匀权重，才能测出 W 加权
    const JointArray ref = {80, -10, 100, -60, 120, -90, 70};
    std::vector<double> leak, amp, leak_rel;
    int bad = 0, zero = 0, big = 0;

    for (int t = 0; t < 300; ++t)
    {
        JointArray q = RandomQ(g);
        for (auto &f : fix) q[f.first] = f.second;
        const Mat4 here = Fk(q);
        const NumericIkStep s = SolveNumericIkStep(kDh, q, here, ref, w, lo, hi, p);
        if (!s.ok) { ++bad; continue; }
        const Jacobian6x7 J = ComputeJacobian(ForwardKinematicsFrames(kDh, q));
        double dqn = 0, zn = 0, lk = 0;
        for (int r = 0; r < 6; ++r)
        {
            double v = 0;
            for (int i = 0; i < 7; ++i) v += J[r][i] * s.dq_deg[i] * 3.14159265358979 / 180.0;
            if (r < 3) lk += v * v;
        }
        lk = std::sqrt(lk);
        for (int i = 0; i < 7; ++i)
        {
            const double dq = s.dq_deg[i] * 3.14159265358979 / 180.0, z = p.nullspace_gain * (ref[i] - q[i]) * 3.14159265358979 / 180.0;
            dqn += w.weight[i] * dq * dq;
            zn += w.weight[i] * z * z;
        }
        dqn = std::sqrt(dqn); zn = std::sqrt(zn);
        if (dqn < 1e-12) { ++zero; continue; }
        leak.push_back(lk);
        leak_rel.push_back(lk / dqn);
        amp.push_back(dqn / zn);
        big += dqn / zn > 1.0 + 1e-3;
    }
    std::printf("%-26s 泄漏‖J dq‖线速度 中位 %.1e 最大 %.1e m | 泄漏/‖dq‖ 最大 %.1e | 放大‖dq‖/‖z‖ 中位 %.3f 最大 %.3f, >1 的 %d 个 | dq=0 %d, !ok %d\n",
                name, Pct(leak, 0.5), Pct(leak, 1.0), Pct(leak_rel, 1.0), Pct(amp, 0.5), Pct(amp, 1.0), big, zero, bad);
}

int main()
{
    std::mt19937 g(9);
    const float s2 = 90.0f;                    // q1: θ2 = q1 − 90
    Sweep("随机构型", {}, g);
    for (float eps : {1.0f, 0.1f, 0.01f, 0.001f, 0.0f})
    {
        char buf[64];
        std::snprintf(buf, sizeof buf, "肩 θ2=%gdeg", eps);
        Sweep(buf, {{1, s2 + eps}}, g);
    }
    for (float eps : {1.0f, 0.1f, 0.01f, 0.001f, 0.0f})
    {
        char buf[64];
        std::snprintf(buf, sizeof buf, "腕 θ6=-180+%gdeg", eps);
        Sweep(buf, {{5, -31.1414f + eps}}, g);
    }
    for (float eps : {1.0f, 0.1f, 0.01f, 0.001f, 0.0f})
    {
        char buf[64];
        std::snprintf(buf, sizeof buf, "肘 θ4=180-%gdeg", eps);
        Sweep(buf, {{3, 9.2463f - eps}}, g);
    }
    Sweep("肩+腕 同时奇异", {{1, 90.0f}, {5, -31.1414f}}, g);
    return 0;
}
