/* 对拍：ArmSim 的数值解（Algorithm/ik_numeric.cpp）与嵌入式 C 端口（Real/DIYcontroller/LibSE/Algorithm/Kinematics/ik_numeric.c）。
 * 同一组随机输入分别喂给两边，比较单步结果，再各自闭环跑一段比较终点。不依赖 MuJoCo。
 * 改任何一边的算法后都要重跑。MSVC 构建（在 vcvars64 环境里，ROOT 为仓库根目录）：
 *   cl /utf-8 /std:c++17 /O2 /EHsc /I%ROOT%\Simulation\ArmSim\Algorithm /I%ROOT%\Real\DIYcontroller\LibSE\Algorithm\Math
 *      ik_numeric_c_golden.cpp %ROOT%\Simulation\ArmSim\Algorithm\{ik_numeric,kinematics,ik_cost}.cpp
 *      %ROOT%\Real\DIYcontroller\LibSE\Algorithm\Kinematics\{ik_numeric,dh}.c
 * 注意 include 顺序：C++ 一侧的 "ik_numeric.h" 必须解析到 ArmSim 的，嵌入式的头文件用相对路径引入。 */
#include <cmath>
#include <cstdio>
#include <random>

#include "ik_numeric.h"
#include "../../../Real/DIYcontroller/LibSE/Algorithm/Kinematics/ik_numeric.h"

static const DhTable kSimDh = {{
    {0.0f, 0.4105f, -0.0897f, -72.5122038f}, {90.0f, 0.0f, 0.0f, -90.0f},        {90.0f, 0.0f, 0.188f, -90.0f},
    {90.0f, 0.0f, -0.0025f, 170.7537190f},   {90.0f, 0.0f, 0.207f, 10.6949890f}, {90.0f, 0.0f, 0.0f, -148.8585943f},
    {90.0f, 0.0f, 0.0f, 0.0f},
}};
/* Real/DIYcontroller/App/module_init.c 的 ARM_DH_J0..J6 */
static const DhTable kRealDh = {{
    {0.0f, 0.0f, 0.0626f, 0.0f}, {90.0f, 0.0f, 0.0f, 0.0f},  {90.0f, 0.0f, -0.148f, 0.0f}, {90.0f, 0.0f, 0.0f, 0.0f},
    {90.0f, 0.0f, 0.1f, 0.0f},   {90.0f, 0.0f, 0.0f, 0.0f}, {90.0f, 0.0f, 0.0f, 0.0f},
}};
static const JointArray kMin = {-90, -20, -175, -110, -175, -120, -175};
static const JointArray kMax = {90, 175, 175, 110, 175, 70, 175};

static void ToC(const DhTable &t, DH_Param_t out[7])
{
    for (int i = 0; i < 7; ++i) out[i] = {t[i].alpha_deg, t[i].a_m, t[i].d_m, t[i].theta_offset_deg};
}
static void ToC(const Mat4 &m, float out[4][4])
{
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) out[r][c] = m(r, c);
}
static NumericIkParam ToSim(const IkNumericParam_t &p)
{
    NumericIkParam s;
    s.damping = p.damping; s.orientation_length_m = p.orientation_length_m; s.nullspace_gain = p.nullspace_gain;
    s.max_step_deg = p.max_step_deg; s.max_pos_err_m = p.max_pos_err_m; s.max_rot_err_rad = p.max_rot_err_rad;
    s.weight_floor = p.weight_floor; s.converged_pos_m = p.converged_pos_m; s.converged_rot_rad = p.converged_rot_rad;
    s.max_limit_passes = p.max_limit_passes; s.max_lead_deg = p.max_lead_deg;
    return s;
}

static int g_fail = 0;
#define CHECK(cond, ...)                                       \
    do {                                                       \
        if (!(cond)) {                                         \
            if (++g_fail <= 15) { std::printf("FAIL %d: ", __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } \
        }                                                      \
    } while (0)

struct Stats { double dq = 0, pos = 0, rot = 0; int mask_diff = 0, flag_diff = 0, ok_diff = 0, limited = 0, cases = 0; };

/* 单步对拍：同一输入两边各算一步 */
static void CompareStep(const DhTable &dh, const JointArray &q, const Mat4 &target, const JointArray &ref,
                        const IkCostWeights &w, const NumericIkParam &sp, const IkNumericParam_t &cp, Stats &st)
{
    DH_Param_t cdh[7];
    float ct[4][4];
    IkNumericStep_t cs;

    ToC(dh, cdh);
    ToC(target, ct);
    const NumericIkStep ss = SolveNumericIkStep(dh, q, target, ref, w, kMin, kMax, sp);
    IkNumeric_Step(cdh, q.data(), ct, ref.data(), w.weight.data(), kMin.data(), kMax.data(), &cp, &cs);

    ++st.cases;
    if (ss.ok != cs.ok) { ++st.ok_diff; return; }
    if (!ss.ok) return;
    for (int i = 0; i < 7; ++i) st.dq = std::fmax(st.dq, std::fabs(ss.dq_deg[i] - cs.dq_deg[i]));
    st.pos = std::fmax(st.pos, std::fabs(ss.pos_err_m - cs.pos_err_m));
    st.rot = std::fmax(st.rot, std::fabs(ss.rot_err_rad - cs.rot_err_rad));
    st.mask_diff += ss.limit_mask != cs.limit_mask;
    st.flag_diff += (ss.step_limited != cs.step_limited) || (ss.converged != cs.converged);
    st.limited += ss.limit_mask != 0;
}

int main()
{
    IkNumericParam_t base;
    IkNumeric_DefaultParam(&base);
    std::mt19937 rng(20261118);
    auto uni = [&](float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); };

    /* 默认参数必须和 ArmSim 的默认值一致 */
    const NumericIkParam sim_default;
    const NumericIkParam from_c = ToSim(base);
    CHECK(sim_default.damping == from_c.damping && sim_default.max_step_deg == from_c.max_step_deg &&
              sim_default.nullspace_gain == from_c.nullspace_gain && sim_default.max_lead_deg == from_c.max_lead_deg &&
              sim_default.max_limit_passes == from_c.max_limit_passes &&
              sim_default.converged_pos_m == from_c.converged_pos_m,
          "default param mismatch");

    /* 1. 单步对拍：两套 DH × 三组参数 × 随机状态。起点三分之一贴近限位，让限位重解的分支被走到 */
    struct Variant { const char *name; const DhTable *dh; };
    const Variant variants[] = {{"sim-dh", &kSimDh}, {"real-dh", &kRealDh}};
    IkNumericParam_t params[3] = {base, base, base};
    params[1].nullspace_gain = 0.5;   // 大增益：零空间项更常触及步长和限位余量
    params[1].max_step_deg = 1.0;
    params[2].damping = 0.005;
    params[2].max_limit_passes = 1;

    for (const Variant &v : variants)
    {
        for (int pi = 0; pi < 3; ++pi)
        {
            Stats st;
            const NumericIkParam sp = ToSim(params[pi]);

            for (int n = 0; n < 4000; ++n)
            {
                JointArray q{}, qt{}, ref{};
                IkCostWeights w{};
                for (int j = 0; j < 7; ++j)
                {
                    q[j] = uni(kMin[j], kMax[j]);
                    if (n % 3 == 0) q[j] = (rng() & 1) ? kMax[j] - uni(0.0f, 0.3f) : kMin[j] + uni(0.0f, 0.3f);
                    qt[j] = uni(kMin[j], kMax[j]);
                    ref[j] = uni(kMin[j], kMax[j]);
                    w.weight[j] = (n % 2) ? uni(0.2f, 5.0f) : 1.0f;
                }
                Mat4 target = ForwardKinematics(*v.dh, qt);
                if (n % 5 == 0) target(0, 3) += uni(-0.5f, 0.5f);   // 部分目标不可达
                CompareStep(*v.dh, q, target, ref, w, sp, params[pi], st);
            }
            std::printf("%-8s param%d: %d cases (limit hit %d)  max|dq|=%.2e deg  pos=%.1e rot=%.1e  mask_diff=%d flag_diff=%d ok_diff=%d\n",
                        v.name, pi, st.cases, st.limited, st.dq, st.pos, st.rot, st.mask_diff, st.flag_diff, st.ok_diff);
            CHECK(st.dq < 1e-4, "%s param%d dq diff %.3e", v.name, pi, st.dq);
            CHECK(st.pos < 1e-6 && st.rot < 1e-6, "%s param%d err diff", v.name, pi);
            CHECK(st.mask_diff == 0 && st.flag_diff == 0 && st.ok_diff == 0, "%s param%d flags differ", v.name, pi);
            CHECK(st.limited > 100, "%s param%d: limit branch barely exercised (%d)", v.name, pi, st.limited);
        }
    }

    /* 2. 闭环对比：两边各自迭代，不互相喂数据，比较终点关节角和收敛周期 */
    for (const Variant &v : variants)
    {
        DH_Param_t cdh[7];
        ToC(*v.dh, cdh);
        const NumericIkParam sp = ToSim(base);
        const IkCostWeights w{};
        double worst_q = 0.0;
        int conv_sim = 0, conv_c = 0, cycle_diff = 0;
        const int n_targets = 60;

        for (int n = 0; n < n_targets; ++n)
        {
            JointArray qt{}, qs{};
            for (int j = 0; j < 7; ++j) qt[j] = uni(kMin[j], kMax[j]);
            const Mat4 target = ForwardKinematics(*v.dh, qt);
            float ct[4][4];
            ToC(target, ct);
            JointArray qc = qs;
            int cyc_sim = -1, cyc_c = -1;

            for (int c = 0; c < 6000 && (cyc_sim < 0 || cyc_c < 0); ++c)
            {
                if (cyc_sim < 0)
                {
                    const NumericIkStep s = SolveNumericIkStep(*v.dh, qs, target, JointArray{}, w, kMin, kMax, sp);
                    if (s.converged) cyc_sim = c;
                    for (int j = 0; j < 7; ++j) qs[j] += s.dq_deg[j];
                }
                if (cyc_c < 0)
                {
                    IkNumericStep_t s;
                    const float zero[7] = {0, 0, 0, 0, 0, 0, 0};
                    IkNumeric_Step(cdh, qc.data(), ct, zero, w.weight.data(), kMin.data(), kMax.data(), &base, &s);
                    if (s.converged) cyc_c = c;
                    for (int j = 0; j < 7; ++j) qc[j] += s.dq_deg[j];
                }
            }
            conv_sim += cyc_sim >= 0;
            conv_c += cyc_c >= 0;
            cycle_diff += cyc_sim != cyc_c;
            for (int j = 0; j < 7; ++j) worst_q = std::fmax(worst_q, std::fabs(qs[j] - qc[j]));
        }
        std::printf("%-8s closed loop: converged sim=%d c=%d of %d, cycle-count differs in %d, max final |dq|=%.2e deg\n",
                    v.name, conv_sim, conv_c, n_targets, cycle_diff, worst_q);
        CHECK(conv_sim == conv_c, "%s convergence count differs", v.name);
        CHECK(cycle_diff <= 1, "%s convergence cycles differ in %d targets", v.name, cycle_diff);
        CHECK(worst_q < 1e-2, "%s final q diverged %.3e", v.name, worst_q);
    }

    std::printf(g_fail ? "FAILED %d\n" : "ALL PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
