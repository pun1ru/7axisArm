/* 数值逆解（Algorithm/ik_numeric）的单元测试，不依赖 MuJoCo。
 * 构建运行：cmake --build build --target ik_numeric_test && ./build/ik_numeric_test */
#include <cmath>
#include <cstdio>

#include "ik_numeric.h"

static const DhTable kDh = {{
    {0.0f, 0.4105f, -0.0897f, -72.5122038f},
    {90.0f, 0.0f, 0.0f, -90.0f},
    {90.0f, 0.0f, 0.188f, -90.0f},
    {90.0f, 0.0f, -0.0025f, 170.7537190f},
    {90.0f, 0.0f, 0.207f, 10.6949890f},
    {90.0f, 0.0f, 0.0f, -148.8585943f},
    {90.0f, 0.0f, 0.0f, 0.0f},
}};
static const JointArray kMin = {-90, -20, -175, -110, -175, -120, -175};
static const JointArray kMax = {90, 175, 175, 110, 175, 70, 175};

static int g_fail = 0;
#define CHECK(cond, ...)                                  \
    do {                                                  \
        if (!(cond)) {                                    \
            ++g_fail;                                     \
            std::printf("FAIL %s:%d ", __FILE__, __LINE__); \
            std::printf(__VA_ARGS__);                     \
            std::printf("\n");                            \
        }                                                 \
    } while (0)


/* 数值 Jacobian 对照解析 Jacobian */
static void TestJacobian()
{
    const JointArray q = {10, 40, -30, 50, 20, -20, 15};
    const Jacobian6x7 j = ComputeJacobian(ForwardKinematicsFrames(kDh, q));
    const Mat4 t0 = ForwardKinematics(kDh, q);
    double worst = 0.0;
    for (int i = 0; i < 7; ++i) {
        JointArray qp = q;
        qp[i] += 0.01f;
        const Mat4 t1 = ForwardKinematics(kDh, qp);
        const auto e = ComputePoseError(t0, t1);
        const double h = 0.01 * 3.14159265358979323846 / 180.0;
        for (int r = 0; r < 6; ++r) worst = std::fmax(worst, std::fabs(e[r] / h - j[r][i]));
    }
    CHECK(worst < 2e-3, "jacobian mismatch %.5f", worst);
    std::printf("jacobian max diff %.2e\n", worst);
}

struct RunResult {
    int cycles;
    double pos_err, rot_err;
    JointArray q;
    double max_step_deg;
    bool limit_ok;
};

static RunResult Run(JointArray q, const Mat4 &target, const JointArray &q_ref, const IkCostWeights &w,
                     const NumericIkParam &p, int max_cycles)
{
    RunResult r{};
    r.limit_ok = true;
    for (int c = 0; c < max_cycles; ++c) {
        const NumericIkStep s = SolveNumericIkStep(kDh, q, target, q_ref, w, kMin, kMax, p);
        if (!s.ok) { r.cycles = -1; return r; }
        r.pos_err = s.pos_err_m;
        r.rot_err = s.rot_err_rad;
        r.cycles = c;
        for (int i = 0; i < 7; ++i) {
            r.max_step_deg = std::fmax(r.max_step_deg, std::fabs(s.dq_deg[i]));
            q[i] += s.dq_deg[i];
            if (q[i] < kMin[i] - 1e-3f || q[i] > kMax[i] + 1e-3f) r.limit_ok = false;
        }
        if (s.converged) break;
    }
    r.q = q;
    return r;
}

int main()
{
    TestJacobian();
    const IkCostWeights w{};
    NumericIkParam p;
    const JointArray home = {0, 60, 0, 60, 0, -30, 0};
    const JointArray goal = {25, 90, -40, 80, 30, -50, 20};
    const Mat4 target = ForwardKinematics(kDh, goal);

    /* 1. 收敛 + 步长 + 限位 */
    RunResult r = Run(home, target, home, w, p, 20000);
    std::printf("converge: cycles=%d pos=%.2e m rot=%.2e rad max_step=%.3f deg\n", r.cycles, r.pos_err, r.rot_err,
                r.max_step_deg);
    CHECK(r.cycles > 0 && r.cycles < 20000, "did not converge (%d)", r.cycles);
    CHECK(r.max_step_deg <= p.max_step_deg + 1e-4, "step %.4f exceeds limit", r.max_step_deg);
    CHECK(r.limit_ok, "joint limit violated");
    const auto fe = ComputePoseError(ForwardKinematics(kDh, r.q), target);
    CHECK(std::fabs(fe[0]) < 1e-3 && std::fabs(fe[1]) < 1e-3 && std::fabs(fe[2]) < 1e-3, "final pose off");

    /* 2. 零空间：到位后末端不动，关节向参考靠拢 */
    JointArray q2 = r.q;
    JointArray far_ref = {0, 20, 60, 100, -60, -90, 80};
    NumericIkParam pn = p;
    pn.nullspace_gain = 0.05;
    const Mat4 hold = ForwardKinematics(kDh, q2);
    double d0 = 0.0;
    for (int i = 0; i < 7; ++i) d0 += std::fabs(q2[i] - far_ref[i]);
    RunResult n = Run(q2, hold, far_ref, w, pn, 4000);
    (void)n;
    JointArray q3 = q2;
    for (int c = 0; c < 4000; ++c) {
        const NumericIkStep s = SolveNumericIkStep(kDh, q3, hold, far_ref, w, kMin, kMax, pn);
        for (int i = 0; i < 7; ++i) q3[i] += s.dq_deg[i];
    }
    double d1 = 0.0;
    for (int i = 0; i < 7; ++i) d1 += std::fabs(q3[i] - far_ref[i]);
    const auto he = ComputePoseError(ForwardKinematics(kDh, q3), hold);
    const double drift = std::sqrt(he[0] * he[0] + he[1] * he[1] + he[2] * he[2]);
    std::printf("nullspace: dist-to-ref %.1f -> %.1f deg, ee drift %.2e m\n", d0, d1, drift);
    CHECK(d1 < d0 - 1.0, "nullspace did not pull toward reference");
    CHECK(drift < 1e-3, "end-effector drifted %.2e", drift);

    /* 3. 目标超出可达：不发散、不越限 */
    Mat4 far = target;
    far(0, 3) += 2.0f;
    RunResult f = Run(home, far, home, w, p, 3000);
    std::printf("unreachable: pos_err=%.3f m, limit_ok=%d\n", f.pos_err, f.limit_ok);
    CHECK(f.limit_ok, "limit violated for unreachable target");
    CHECK(f.cycles >= 0, "solver failed on unreachable target");

    /* 4. 权重：给 J0 很大权重，J0 应该动得比默认少 */
    IkCostWeights heavy{};
    heavy.weight[0] = 100.0f;
    RunResult a = Run(home, target, home, w, p, 20000);
    RunResult b = Run(home, target, home, heavy, p, 20000);
    std::printf("weights: |dJ0| default=%.2f heavy=%.2f deg\n", std::fabs(a.q[0] - home[0]),
                std::fabs(b.q[0] - home[0]));
    CHECK(std::fabs(b.q[0] - home[0]) < std::fabs(a.q[0] - home[0]), "heavy weight did not reduce J0 motion");

    /* 5. 起点贴着限位：必须被钳住且不越限 */
    JointArray edge = home;
    edge[1] = 174.9f;
    RunResult g = Run(edge, target, home, w, p, 5000);
    CHECK(g.limit_ok, "limit violated from edge start");

    /* 6. 非法输入 */
    IkCostWeights bad{};
    bad.weight.fill(0.0f);
    CHECK(!SolveNumericIkStep(kDh, home, target, home, bad, kMin, kMax, p).ok, "zero weights accepted");

    std::printf(g_fail ? "FAILED %d\n" : "ALL PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
