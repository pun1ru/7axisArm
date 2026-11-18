/* 同一批可达目标上比较零空间实现：收敛数、周期数、每周期关节增量的突变（平滑度）、求解失败次数 */
#include <cstdlib>
#include "common.h"
int main(int argc, char **argv)
{
    const int n = argc > 1 ? std::atoi(argv[1]) : 600;
    std::mt19937 g(12345);
    const JointArray start{};
    const IkCostWeights w{};
    NumericIkParam p;
    int conv = 0, fails = 0, near_sing = 0, near_sing_conv = 0;
    std::vector<double> cyc, jerk;

    for (int t = 0; t < n; ++t)
    {
        const JointArray qt = RandomQ(g);
        const Mat4 target = Fk(qt);
        const bool ns = std::fabs(std::sin((qt[1] - 90.0f) * kDegToRad)) < 0.1 || std::fabs(std::sin((qt[5] - 148.86f) * kDegToRad)) < 0.1;
        JointArray q = start, prev{};
        double worst = 0.0;
        bool ok_conv = false;
        for (int c = 0; c < 6000; ++c)
        {
            const NumericIkStep s = SolveNumericIkStep(kDh, q, target, start, w, kMin, kMax, p);
            if (!s.ok) { ++fails; break; }
            if (s.converged) { ok_conv = true; cyc.push_back(c); break; }
            double d = 0;
            for (int i = 0; i < kJointCount; ++i) { d += (s.dq_deg[i] - prev[i]) * (s.dq_deg[i] - prev[i]); q[i] += s.dq_deg[i]; prev[i] = s.dq_deg[i]; }
            if (c > 0) worst = std::max(worst, std::sqrt(d));
        }
        conv += ok_conv;
        near_sing += ns;
        near_sing_conv += ns && ok_conv;
        jerk.push_back(worst);
    }
    std::printf("到位 %d/%d | 靠近肩/腕奇异的目标 %d 个, 到位 %d | 求解失败(!ok) %d 次 | 周期 中位 %.0f p95 %.0f | "
                "相邻周期 dq 突变(deg) 中位 %.3f p99 %.3f 最大 %.3f\n",
                conv, n, near_sing, near_sing_conv, fails, Pct(cyc, 0.5), Pct(cyc, 0.95), Pct(jerk, 0.5), Pct(jerk, 0.99), Pct(jerk, 1.0));
}
