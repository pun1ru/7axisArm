/* 诊断：对数值解未到位的可达目标，看它是停住了、还是仍在慢慢走；换参数会不会好。 */
#include "common.h"

struct Tail {
    double peak_dq = 0;      // 最后 50 个周期里最大关节增量 deg
    uint8_t limit_mask = 0;
    bool step_limited = false;
};

static NumRun Run(const JointArray &start, const Mat4 &t, const NumericIkParam &p, int cap, Tail &tail)
{
    NumRun r;
    JointArray q = start;
    const IkCostWeights w{};
    for (int c = 0; c < cap; ++c)
    {
        const NumericIkStep s = SolveNumericIkStep(kDh, q, t, start, w, kMin, kMax, p);
        r.cycles = c;
        if (c >= cap - 50)
        {
            tail.limit_mask |= s.limit_mask;
            tail.step_limited |= s.step_limited;
            for (int i = 0; i < kJointCount; ++i) tail.peak_dq = std::max(tail.peak_dq, double(std::fabs(s.dq_deg[i])));
        }
        if (s.converged) { r.converged = true; break; }
        for (int i = 0; i < kJointCount; ++i) q[i] += s.dq_deg[i];
    }
    r.q = q;
    r.pos = PosErr(Fk(q), t);
    r.rot_deg = RotErrDeg(Fk(q), t);
    return r;
}

int main()
{
    const JointArray start{};
    std::mt19937 g(12345);
    NumericIkParam base;
    struct Variant { const char *name; NumericIkParam p; int cap; };
    std::vector<Variant> vs;
    vs.push_back({"default cap8000", base, 8000});
    vs.push_back({"default cap40000", base, 40000});
    NumericIkParam lowd = base; lowd.damping = 0.005;
    vs.push_back({"damping 0.005 cap8000", lowd, 8000});
    NumericIkParam nonull = base; nonull.nullspace_gain = 0.0;
    vs.push_back({"nullspace off cap8000", nonull, 8000});
    NumericIkParam big = base; big.max_pos_err_m = 0.1; big.max_rot_err_rad = 0.5; big.max_step_deg = 0.5;
    vs.push_back({"bigger err/step cap8000", big, 8000});

    std::vector<JointArray> targets_q;
    for (int i = 0; i < 300; ++i) targets_q.push_back(RandomQ(g));

    for (const Variant &v : vs)
    {
        int ok = 0, stalled = 0, moving = 0;
        std::vector<double> peak;
        for (const JointArray &qt : targets_q)
        {
            Tail tail;
            const NumRun r = Run(start, Fk(qt), v.p, v.cap, tail);
            if (r.converged) { ++ok; continue; }
            (tail.peak_dq < 1e-3 ? stalled : moving)++;
            peak.push_back(tail.peak_dq);
        }
        std::printf("%-26s 到位 %3d/300  未到位: 停住(末 50 周期 dq<1e-3°) %3d, 仍在走 %3d, 末段 dq 中位 %.4f deg\n", v.name, ok,
                    stalled, moving, Pct(peak, 0.5));
    }
    return 0;
}
