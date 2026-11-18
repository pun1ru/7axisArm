/* 实验 A：目标 = 随机关节角的正解，一定可达。比较解析解（当前分支 / 任意分支）和数值解的覆盖率与精度。 */
#include <cstdlib>

#include "common.h"

int main(int argc, char **argv)
{
    const int n = argc > 1 ? std::atoi(argv[1]) : 600;
    const int start_id = argc > 2 ? std::atoi(argv[2]) : 0;
    const JointArray starts[2] = {{0, 0, 0, 0, 0, 0, 0}, {0, 60, 0, 60, 0, -30, 0}};
    const JointArray start = starts[start_id];
    const Branch start_branch = ComputeBranch(kDh, start);
    const IkCostWeights w{};
    NumericIkParam p;
    std::mt19937 g(12345);

    int tab[2][2][2] = {};   // [同分支解析][任意分支解析][数值]
    int total_sing = 0;
    std::vector<double> cyc, num_pos, num_rot, ana_pos;
    std::vector<double> fail_pos, fail_rot;
    int fail_limit = 0, fail_other = 0, num_nonconv_close = 0;
    int num_only = 0, num_only_branch_changed = 0, ana_only = 0, branch_changed = 0;

    for (int i = 0; i < n; ++i)
    {
        const JointArray q_true = RandomQ(g);
        const Mat4 target = Fk(q_true);
        JointArray qa{};
        const bool a_same = AnalyticBranch(target, start_branch, start, qa);
        if (a_same) ana_pos.push_back(PosErr(Fk(qa), target));
        JointArray qb{};
        const bool a_any = a_same || AnalyticAny(target, start, qb);
        const NumRun r = RunNumeric(start, target, w, p, 8000);
        ++tab[a_same][a_any][r.converged];
        num_only += r.converged && !a_same;
        ana_only += !r.converged && a_same;
        if (r.converged)
        {
            const bool changed = ComputeBranch(kDh, r.q).index() != start_branch.index();
            branch_changed += changed;
            num_only_branch_changed += changed && !a_same;
            cyc.push_back(r.cycles);
            num_pos.push_back(r.pos);
            num_rot.push_back(r.rot_deg);
        }
        else
        {
            fail_pos.push_back(r.pos);
            fail_rot.push_back(r.rot_deg);
            (r.limit_blocked ? fail_limit : fail_other)++;
            if (r.pos < 0.005 && r.rot_deg < 1.0) ++num_nonconv_close;
        }
        const double s2 = std::fabs(std::sin((q_true[1] + kDh[1].theta_offset_deg) * kDegToRad));
        const double s6 = std::fabs(std::sin((q_true[5] + kDh[5].theta_offset_deg) * kDegToRad));
        if (s2 < 0.02 || s6 < 0.02) ++total_sing;
    }

    int a_same_n = 0, a_any_n = 0, num_n = 0;
    for (int s = 0; s < 2; ++s)
        for (int a = 0; a < 2; ++a)
            for (int c = 0; c < 2; ++c)
            {
                if (s) a_same_n += tab[s][a][c];
                if (a) a_any_n += tab[s][a][c];
                if (c) num_n += tab[s][a][c];
            }
    std::printf("== 实验 A：%d 个可达目标，起点 %d，起点分支 %d ==\n", n, start_id, start_branch.index());
    std::printf("解析解 当前分支: %5.1f%%   解析解 任意分支: %5.1f%%   数值解: %5.1f%%\n",
                100.0 * a_same_n / n, 100.0 * a_any_n / n, 100.0 * num_n / n);
    const int same_total = tab[1][1][0] + tab[1][1][1];
    const int anyonly = tab[0][1][0] + tab[0][1][1];
    const int none = tab[0][0][0] + tab[0][0][1];
    std::printf("当前分支解析解可达(%d): 数值解到位 %d (%.1f%%)\n", same_total, tab[1][1][1],
                same_total ? 100.0 * tab[1][1][1] / same_total : 0.0);
    std::printf("仅其他分支解析解可达(%d): 数值解到位 %d (%.1f%%)\n", anyonly, tab[0][1][1],
                anyonly ? 100.0 * tab[0][1][1] / anyonly : 0.0);
    std::printf("解析解（含 ψ 采样 / 非奇异过滤）全部找不到但目标确实可达(%d): 数值解到位 %d (%.1f%%)\n", none,
                tab[0][0][1], none ? 100.0 * tab[0][0][1] / none : 0.0);
    std::printf("数值解到位而当前分支解析解到不了: %d (其中数值解最终分支与起点不同 %d)；当前分支解析解可达而数值解没到位: %d；数值解到位且分支改变共 %d\n",
                num_only, num_only_branch_changed, ana_only, branch_changed);
    std::printf("目标靠近肩/腕奇异 (|sinθ2| 或 |sinθ6| < 0.02): %d\n", total_sing);
    std::printf("解析解误差(开环, float 精度) 位置 中位 %.2e m 最大 %.2e m\n", Pct(ana_pos, 0.5), Pct(ana_pos, 1.0));
    std::printf("数值解到位的: 周期数 中位 %.0f p95 %.0f 最大 %.0f (2ms/周期 → 中位 %.2f s)\n", Pct(cyc, 0.5),
                Pct(cyc, 0.95), Pct(cyc, 1.0), Pct(cyc, 0.5) * 0.002);
    std::printf("                位置误差 中位 %.2e 最大 %.2e m, 姿态误差 最大 %.3f deg\n", Pct(num_pos, 0.5),
                Pct(num_pos, 1.0), Pct(num_rot, 1.0));
    std::printf("数值解未到位 %zu 个: 限位卡住 %d, 其他(局部极小/奇异附近) %d, 其中残差<5mm且<1° %d\n",
                fail_pos.size(), fail_limit, fail_other, num_nonconv_close);
    if (!fail_pos.empty())
        std::printf("  未到位残差 位置 中位 %.3f m 最大 %.3f m, 姿态 中位 %.1f deg 最大 %.1f deg\n", Pct(fail_pos, 0.5),
                    Pct(fail_pos, 1.0), Pct(fail_rot, 0.5), Pct(fail_rot, 1.0));
    return 0;
}
