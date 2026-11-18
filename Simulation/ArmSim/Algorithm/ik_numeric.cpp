#include "ik_numeric.h"

#include <algorithm>
#include <cmath>

namespace {

using Vec6 = std::array<double, 6>;
using Vec7 = std::array<double, kJointCount>;
using Mat6 = std::array<std::array<double, 6>, 6>;

constexpr double kDegToRadD = 3.14159265358979323846 / 180.0;
constexpr double kRadToDegD = 180.0 / 3.14159265358979323846;

/* 解 A·x = b，A 对称正定（Cholesky）。A 不正定（主元 ≤ 0 或非有限）返回 false */
bool SolveSpd6(Mat6 a, const Vec6 &b, Vec6 &x)
{
    for (int j = 0; j < 6; ++j)
    {
        double diag = a[j][j];
        for (int k = 0; k < j; ++k)
        {
            diag -= a[j][k] * a[j][k];
        }
        if (!(diag > 1e-18) || !std::isfinite(diag))
        {
            return false;
        }
        a[j][j] = std::sqrt(diag);
        for (int i = j + 1; i < 6; ++i)
        {
            double sum = a[i][j];
            for (int k = 0; k < j; ++k)
            {
                sum -= a[i][k] * a[j][k];
            }
            a[i][j] = sum / a[j][j];
        }
    }
    Vec6 y{};
    for (int i = 0; i < 6; ++i)
    {
        double sum = b[i];
        for (int k = 0; k < i; ++k)
        {
            sum -= a[i][k] * y[k];
        }
        y[i] = sum / a[i][i];
    }
    for (int i = 5; i >= 0; --i)
    {
        double sum = y[i];
        for (int k = i + 1; k < 6; ++k)
        {
            sum -= a[k][i] * x[k];
        }
        x[i] = sum / a[i][i];
    }
    return true;
}

double Norm3(const Vec6 &v, int offset)
{
    return std::sqrt(v[offset] * v[offset] + v[offset + 1] * v[offset + 1] + v[offset + 2] * v[offset + 2]);
}

double Det6(std::array<std::array<double, 6>, 6> a)
{
    double det = 1.0;

    for (int c = 0; c < 6; ++c)
    {
        int pivot = c;
        for (int r = c + 1; r < 6; ++r)
        {
            if (std::fabs(a[r][c]) > std::fabs(a[pivot][c])) pivot = r;
        }
        if (std::fabs(a[pivot][c]) < 1e-300) return 0.0;
        if (pivot != c)
        {
            std::swap(a[pivot], a[c]);
            det = -det;
        }
        det *= a[c][c];
        for (int r = c + 1; r < 6; ++r)
        {
            const double f = a[r][c] / a[c][c];
            for (int k = c; k < 6; ++k) a[r][k] -= f * a[c][k];
        }
    }
    return det;
}

/* 七个关节全自由时 J（6×7）的零空间向量 n：广义叉积 n_i = (−1)^i·det(J 删去第 i 列)，
 * 因此 J·n = 0 严格成立，不依赖阻尼。‖n‖ = √det(J Jᵀ) 是可操作度，奇异时趋于 0（此时零空间退化，返回 0）。 */
double NullVector(const Jacobian6x7 &jac, Vec7 &n)
{
    double sum_sq = 0.0;

    for (int skip = 0; skip < kJointCount; ++skip)
    {
        Mat6 minor{};
        int col = 0;
        for (int c = 0; c < kJointCount; ++c)
        {
            if (c == skip) continue;
            for (int r = 0; r < 6; ++r)
            {
                minor[static_cast<size_t>(r)][static_cast<size_t>(col)] = jac[static_cast<size_t>(r)][static_cast<size_t>(c)];
            }
            ++col;
        }
        n[static_cast<size_t>(skip)] = ((skip & 1) ? -1.0 : 1.0) * Det6(minor);
        sum_sq += n[static_cast<size_t>(skip)] * n[static_cast<size_t>(skip)];
    }
    return std::sqrt(sum_sq);
}

/* 自由关节集合上的加权阻尼解：dq_task = J̄e，J̄ = W⁻¹Jᵀ(J W⁻¹ Jᵀ + λ²I)⁻¹。
 * dq_null 是 z 在 J 零空间上的 W 加权投影：七个关节全自由且可操作度够大时才有（1 维零空间，精确投影），
 * 否则为 0。不用 (I − J̄J)：J̄ 带阻尼时它不是投影，会把 z 漏进任务空间，远离参考构型时把任务项顶住。
 * 固定关节（free[i] = false）不参与，两个输出里对应位置为 0。 */
bool SolveFreeSet(const Jacobian6x7 &jac, const Vec7 &w_inv, const std::array<bool, kJointCount> &is_free,
                  const Vec6 &e, const Vec7 &z, double damping, Vec7 &dq_task, Vec7 &dq_null)
{
    Mat6 a{};
    Vec6 y{};
    bool all_free = true;

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            double sum = 0.0;
            for (int i = 0; i < kJointCount; ++i)
            {
                if (is_free[static_cast<size_t>(i)])
                {
                    sum += w_inv[static_cast<size_t>(i)] * jac[r][i] * jac[c][i];
                }
            }
            a[r][c] = sum + (r == c ? damping * damping : 0.0);
        }
    }
    if (!SolveSpd6(a, e, y))
    {
        return false;
    }
    for (int i = 0; i < kJointCount; ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        double task = 0.0;

        all_free = all_free && is_free[idx];
        dq_null[idx] = 0.0;
        if (!is_free[idx])
        {
            dq_task[idx] = 0.0;
            continue;
        }
        for (int r = 0; r < 6; ++r)
        {
            task += jac[r][i] * y[r];
        }
        dq_task[idx] = w_inv[idx] * task;
    }

    /* 零空间项：7 个关节全自由时零空间 1 维 = span(n)，z 在其上做 W 加权投影 c·n，c = nᵀWz / nᵀWn。
     * 有关节被限位固定后只剩 6 个自由关节，没有冗余度，零空间项为 0 */
    Vec7 n{};
    if (all_free && NullVector(jac, n) > 1e-9)
    {
        double num = 0.0;
        double den = 0.0;

        for (int i = 0; i < kJointCount; ++i)
        {
            const size_t idx = static_cast<size_t>(i);
            const double w = 1.0 / w_inv[idx];

            num += n[idx] * w * z[idx];
            den += n[idx] * w * n[idx];
        }
        if (den > 0.0)
        {
            for (int i = 0; i < kJointCount; ++i)
            {
                dq_null[static_cast<size_t>(i)] = n[static_cast<size_t>(i)] * (num / den);
            }
        }
    }
    return true;
}

}  // namespace

bool IsValidNumericIkParam(const NumericIkParam &p)
{
    const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };

    return positive(p.damping) && positive(p.orientation_length_m) && std::isfinite(p.nullspace_gain) &&
           p.nullspace_gain >= 0.0 && positive(p.max_step_deg) && positive(p.max_pos_err_m) &&
           positive(p.max_rot_err_rad) && positive(p.weight_floor) && p.converged_pos_m >= 0.0 &&
           p.converged_rot_rad >= 0.0 && p.max_limit_passes >= 0;
}

Jacobian6x7 ComputeJacobian(const FrameArray &frames)
{
    Jacobian6x7 jac{};
    const Mat4 &tool = frames.back();

    for (int i = 0; i < kJointCount; ++i)
    {
        const Mat4 &f = frames[static_cast<size_t>(i)];
        const double z[3] = {f(0, 2), f(1, 2), f(2, 2)};
        const double r[3] = {static_cast<double>(tool(0, 3)) - f(0, 3),
                             static_cast<double>(tool(1, 3)) - f(1, 3),
                             static_cast<double>(tool(2, 3)) - f(2, 3)};

        jac[0][i] = z[1] * r[2] - z[2] * r[1];
        jac[1][i] = z[2] * r[0] - z[0] * r[2];
        jac[2][i] = z[0] * r[1] - z[1] * r[0];
        jac[3][i] = z[0];
        jac[4][i] = z[1];
        jac[5][i] = z[2];
    }
    return jac;
}

std::array<double, 6> ComputePoseError(const Mat4 &current, const Mat4 &desired)
{
    Vec6 e{};

    for (int k = 0; k < 3; ++k)
    {
        e[static_cast<size_t>(k)] = static_cast<double>(desired(k, 3)) - current(k, 3);
    }
    for (int k = 0; k < 3; ++k)   // 第 k 列：r_c × r_d
    {
        const double c[3] = {current(0, k), current(1, k), current(2, k)};
        const double d[3] = {desired(0, k), desired(1, k), desired(2, k)};

        e[3] += 0.5 * (c[1] * d[2] - c[2] * d[1]);
        e[4] += 0.5 * (c[2] * d[0] - c[0] * d[2]);
        e[5] += 0.5 * (c[0] * d[1] - c[1] * d[0]);
    }
    return e;
}

NumericIkStep SolveNumericIkStep(const DhTable &table, const JointArray &q_deg, const Mat4 &target,
                                 const JointArray &q_ref_deg, const IkCostWeights &weights,
                                 const JointArray &limit_min_deg, const JointArray &limit_max_deg,
                                 const NumericIkParam &param)
{
    NumericIkStep out;
    Vec7 w_inv{};
    Vec7 z{};
    Vec7 lo{};   // 本周期关节增量的下界、上界（rad），区间一定含 0
    Vec7 hi{};

    out.ok = false;
    if (!IsValidNumericIkParam(param) || !IsValidIkCostWeights(weights))
    {
        return out;
    }
    for (int i = 0; i < kJointCount; ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        const double q = q_deg[idx];

        if (!std::isfinite(q) || !std::isfinite(q_ref_deg[idx]) || !(limit_min_deg[idx] < limit_max_deg[idx]))
        {
            return out;
        }
        w_inv[idx] = 1.0 / std::max(static_cast<double>(weights.weight[idx]), param.weight_floor);
        z[idx] = param.nullspace_gain * (static_cast<double>(q_ref_deg[idx]) - q) * kDegToRadD;
        lo[idx] = std::min(0.0, (static_cast<double>(limit_min_deg[idx]) - q) * kDegToRadD);
        hi[idx] = std::max(0.0, (static_cast<double>(limit_max_deg[idx]) - q) * kDegToRadD);
    }

    const FrameArray frames = ForwardKinematicsFrames(table, q_deg);
    Jacobian6x7 jac = ComputeJacobian(frames);
    Vec6 e = ComputePoseError(frames.back(), target);
    for (double v : e)
    {
        if (!std::isfinite(v))
        {
            return out;
        }
    }

    out.pos_err_m = Norm3(e, 0);
    out.rot_err_rad = Norm3(e, 3);
    out.converged = out.pos_err_m <= param.converged_pos_m && out.rot_err_rad <= param.converged_rot_rad;

    /* 单步误差限幅，再把姿态行折算成位置量纲 */
    const double pos_scale = out.pos_err_m > param.max_pos_err_m ? param.max_pos_err_m / out.pos_err_m : 1.0;
    const double rot_scale = out.rot_err_rad > param.max_rot_err_rad ? param.max_rot_err_rad / out.rot_err_rad : 1.0;
    for (int r = 0; r < 3; ++r)
    {
        e[static_cast<size_t>(r)] *= pos_scale;
        e[static_cast<size_t>(r + 3)] *= rot_scale * param.orientation_length_m;
        for (int i = 0; i < kJointCount; ++i)
        {
            jac[static_cast<size_t>(r + 3)][static_cast<size_t>(i)] *= param.orientation_length_m;
        }
    }

    /* 限位：越界的关节钳到边界并固定，它的贡献从误差里扣掉，剩下的关节重新解 */
    std::array<bool, kJointCount> is_free;
    Vec7 fixed{};
    Vec7 dq_task{};
    Vec7 dq_null{};

    is_free.fill(true);
    for (int pass = 0; pass <= param.max_limit_passes; ++pass)
    {
        Vec6 residual = e;
        bool violated = false;
        bool any_free = false;

        for (int i = 0; i < kJointCount; ++i)
        {
            if (!is_free[static_cast<size_t>(i)])
            {
                for (int r = 0; r < 6; ++r)
                {
                    residual[static_cast<size_t>(r)] -= jac[static_cast<size_t>(r)][static_cast<size_t>(i)] *
                                                        fixed[static_cast<size_t>(i)];
                }
            }
        }
        if (!SolveFreeSet(jac, w_inv, is_free, residual, z, param.damping, dq_task, dq_null))
        {
            return out;
        }
        if (pass == param.max_limit_passes)
        {
            break;   // 轮数用完：不再冻结，后面统一钳位
        }
        for (int i = 0; i < kJointCount; ++i)
        {
            const size_t idx = static_cast<size_t>(i);
            const double total = dq_task[idx] + dq_null[idx];

            if (!is_free[idx])
            {
                continue;
            }
            if (total < lo[idx] || total > hi[idx])
            {
                is_free[idx] = false;
                fixed[idx] = total < lo[idx] ? lo[idx] : hi[idx];
                out.limit_mask = static_cast<uint8_t>(out.limit_mask | (1U << i));
                violated = true;
            }
            else
            {
                any_free = true;
            }
        }
        if (!violated || !any_free)
        {
            if (violated)   // 全部冻结：dq 就是各自的边界值
            {
                dq_task.fill(0.0);
                dq_null.fill(0.0);
            }
            break;
        }
    }
    for (int i = 0; i < kJointCount; ++i)
    {
        if (!is_free[static_cast<size_t>(i)])
        {
            dq_task[static_cast<size_t>(i)] = fixed[static_cast<size_t>(i)];
        }
    }

    /* 步长约束：任务项优先。任务项超步长时整体等比缩小（保持方向），零空间项只用剩下的余量 */
    const double step = param.max_step_deg * kDegToRadD;
    double task_peak = 0.0;
    double null_scale = 1.0;

    for (double v : dq_task)
    {
        task_peak = std::max(task_peak, std::fabs(v));
    }
    if (task_peak > step)
    {
        const double shrink = step / task_peak;
        for (double &v : dq_task)
        {
            v *= shrink;
        }
        out.step_limited = true;
    }
    for (int i = 0; i < kJointCount; ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        const double low = std::max(-step, lo[idx]);
        const double high = std::min(step, hi[idx]);

        if (dq_null[idx] > 0.0)
        {
            null_scale = std::min(null_scale, (high - dq_task[idx]) / dq_null[idx]);
        }
        else if (dq_null[idx] < 0.0)
        {
            null_scale = std::min(null_scale, (low - dq_task[idx]) / dq_null[idx]);
        }
    }
    null_scale = std::max(0.0, std::min(1.0, null_scale));

    for (int i = 0; i < kJointCount; ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        const double dq = std::max(lo[idx], std::min(hi[idx], dq_task[idx] + null_scale * dq_null[idx]));

        out.dq_deg[idx] = static_cast<float>(dq * kRadToDegD);
    }
    out.ok = true;
    return out;
}
