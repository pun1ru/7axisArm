/* 实验 B：解析解到不了的目标，数值解会不会尽力逼近？
 *  B1 超出臂展：腕心沿肩→腕方向拉到 Rmax + d 处，姿态不变。解析解所有分支都应报 OutOfReach。
 *  B2 位置来自一个随机构型、姿态来自另一个随机构型：位置可达、姿态可能不可行。 */
#include <cstdlib>

#include "common.h"

static Mat4 PushedTarget(const Mat4 &t, double extra, double &rmax_out)
{
    const SrsParam &s = Srs();
    const double reach_sq = (s.upper_arm_m + s.forearm_m) * (s.upper_arm_m + s.forearm_m) + s.elbow_offset_m * s.elbow_offset_m;
    const double rmax = std::sqrt(reach_sq);
    double w[3], u[3], len = 0;

    for (int i = 0; i < 3; ++i)
    {
        w[i] = t(i, 3) - s.tool_m * t(i, 2);
        u[i] = w[i] - s.shoulder_m[static_cast<size_t>(i)];
        len += u[i] * u[i];
    }
    len = std::sqrt(len);
    Mat4 out = t;
    for (int i = 0; i < 3; ++i)
    {
        const double wn = s.shoulder_m[static_cast<size_t>(i)] + u[i] / len * (rmax + extra);
        out(i, 3) = static_cast<float>(wn + s.tool_m * t(i, 2));
    }
    rmax_out = rmax;
    return out;
}

static double WristDist(const Mat4 &t)
{
    const SrsParam &s = Srs();
    double len = 0;
    for (int i = 0; i < 3; ++i)
    {
        const double d = t(i, 3) - s.tool_m * t(i, 2) - s.shoulder_m[static_cast<size_t>(i)];
        len += d * d;
    }
    return std::sqrt(len);
}

int main(int argc, char **argv)
{
    const int n = argc > 1 ? std::atoi(argv[1]) : 200;
    const JointArray start{};
    const IkCostWeights w{};
    NumericIkParam p;
    std::mt19937 g(777);

    std::printf("== 实验 B1：超出臂展 d 米（姿态不变），起点 q=0 ==\n");
    for (double d : {0.02, 0.05, 0.10, 0.20})
    {
        int ana_ok = 0, in_limit = 0, extended = 0, closer = 0;
        std::vector<double> ratio, rot, dist_ratio;
        double rmax = 0;
        for (int i = 0; i < n; ++i)
        {
            const Mat4 t = PushedTarget(Fk(RandomQ(g)), d, rmax);
            JointArray tmp{};
            ana_ok += AnalyticAny(t, start, tmp);
            const double e0 = PosErr(Fk(start), t);
            const NumRun r = RunNumeric(start, t, w, p, 6000);
            bool lim = true;
            for (int j = 0; j < kJointCount; ++j) lim = lim && r.q[j] >= kMin[j] - 1e-3f && r.q[j] <= kMax[j] + 1e-3f;
            in_limit += lim;
            ratio.push_back(r.pos / d);
            rot.push_back(r.rot_deg);
            dist_ratio.push_back(WristDist(Fk(r.q)) / rmax);
            extended += WristDist(Fk(r.q)) / rmax > 0.99;
            closer += r.pos < e0;
        }
        std::printf("d=%.2f m: 解析解可达 %d/%d | 数值解 末端位置误差/d 中位 %.2f p90 %.2f | 腕心到肩距离/Rmax 中位 %.3f, 伸直(>0.99) %d/%d | "
                    "姿态残差 中位 %.1f° | 比起点更近 %d/%d | 不越限 %d/%d\n",
                    d, ana_ok, n, Pct(ratio, 0.5), Pct(ratio, 0.9), Pct(dist_ratio, 0.5), extended, n, Pct(rot, 0.5), closer, n,
                    in_limit, n);
    }

    std::printf("\n== 实验 B2：位置来自构型 1、姿态来自构型 2（位置可达，姿态未必可行）==\n");
    int feasible = 0, infeasible = 0, f_ok = 0, i_ok = 0;
    std::vector<double> ipos, irot, fpos;
    for (int i = 0; i < n; ++i)
    {
        Mat4 t = Fk(RandomQ(g));
        const Mat4 o = Fk(RandomQ(g));
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) t(r, c) = o(r, c);
        JointArray tmp{};
        const bool ana = AnalyticAny(t, start, tmp);
        const NumRun r = RunNumeric(start, t, w, p, 6000);
        if (ana) { ++feasible; f_ok += r.converged; fpos.push_back(r.pos); }
        else { ++infeasible; i_ok += r.converged; ipos.push_back(r.pos); irot.push_back(r.rot_deg); }
    }
    std::printf("解析解（任意分支）能到: %d, 其中数值解到位 %d (%.1f%%)\n", feasible, f_ok, feasible ? 100.0 * f_ok / feasible : 0.0);
    std::printf("解析解到不了: %d, 其中数值解竟然到位 %d\n", infeasible, i_ok);
    if (!ipos.empty())
    {
        int pos_kept = 0;
        for (double v : ipos) pos_kept += v < 0.005;
        std::printf("  数值解残差: 位置 中位 %.1f mm p90 %.1f mm, 姿态 中位 %.1f° p90 %.1f° | 位置误差<5mm 的占 %d/%zu\n",
                    Pct(ipos, 0.5) * 1000, Pct(ipos, 0.9) * 1000, Pct(irot, 0.5), Pct(irot, 0.9), pos_kept, ipos.size());
    }
    return 0;
}
