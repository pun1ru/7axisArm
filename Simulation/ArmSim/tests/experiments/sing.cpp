/* 在精确奇异 / 近奇异位形上：目标 = 当前位姿（任务误差为 0），参考构型不同，只有零空间项在起作用。
 * 正确的投影：末端不应该动。比较一步后的末端漂移和关节增量。 */
#include "common.h"
int main()
{
    std::mt19937 g(5);
    struct Case { const char *name; int joint; float value; };
    const Case cases[] = {{"肩奇异 θ2=0 (q1=90)", 1, 90.0f}, {"肩近奇异 θ2=0.01°", 1, 90.01f}, {"肩近奇异 θ2=1°", 1, 91.0f},
                          {"腕奇异 θ6=-180 (q5=-31.14)", 5, -31.1414f}, {"腕近奇异 θ6≈-180+0.01°", 5, -31.1314f},
                          {"肘伸直 θ4=180 (q3=9.25)", 3, 9.2463f}, {"肘近伸直 +0.01°", 3, 9.2563f}};
    NumericIkParam p;
    p.nullspace_gain = 0.5;       // 放大零空间项，让漂移更容易看出来
    p.max_step_deg = 5.0;         // 放宽步长限幅，避免把差异压掉
    const IkCostWeights w{};
    std::printf("%-30s %12s %12s %12s %12s\n", "位形(200 组随机其余关节)", "漂移中位 m", "漂移最大 m", "|dq|最大 deg", "!ok 次数");
    for (const Case &c : cases)
    {
        std::vector<double> drift, dqn;
        int bad = 0;
        for (int t = 0; t < 200; ++t)
        {
            JointArray q = RandomQ(g);
            q[c.joint] = c.value;
            const Mat4 here = Fk(q);
            const NumericIkStep s = SolveNumericIkStep(kDh, q, here, JointArray{}, w, kMin, kMax, p);
            if (!s.ok) { ++bad; continue; }
            JointArray q2 = q;
            double nrm = 0;
            for (int i = 0; i < kJointCount; ++i) { q2[i] += s.dq_deg[i]; nrm = std::max(nrm, double(std::fabs(s.dq_deg[i]))); }
            drift.push_back(PosErr(here, Fk(q2)));
            dqn.push_back(nrm);
        }
        std::printf("%-30s %12.2e %12.2e %12.3f %12d\n", c.name, Pct(drift, 0.5), Pct(drift, 1.0), Pct(dqn, 1.0), bad);
    }
}
