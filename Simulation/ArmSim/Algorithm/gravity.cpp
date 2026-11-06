#include "gravity.h"

#include <cmath>

/* U = g * sum_i row2(T_i) . p_i，T_i = A_0 A_1 ... A_i，p_i = [m*cx m*cy m*cz m]
 * tau_j = dU/dq_j = g * sum_{i>=j} row2(T_{j-1}) * dA_j * A_{j+1}...A_i . p_i
 * 只用 T 的第 3 行（世界 z），所以只递推一个 1x4 行向量 v：
 *     v0 = row2(T_{j-1}) * dA_j,  v = v * A_i (i = j+1..),  tau_j += g * (v . p_i)
 * dA_j 由 A_j 直接得到：前两列互换并取负，后两列与 q 无关。 */

namespace {

using Row = std::array<float, 4>;

Row RowMulMat(const Row &in, const Mat4 &M)
{
    Row out;
    for (int c = 0; c < 4; ++c)
    {
        out[c] = in[0] * M(0, c) + in[1] * M(1, c) + in[2] * M(2, c) + in[3] * M(3, c);
    }
    return out;
}

/* row * dA；dA 的第 3、4 列恒为 0 */
Row RowMulDA(const Row &row, const Mat4 &A)
{
    return {row[0] * A(0, 1) + row[1] * A(1, 1) + row[2] * A(2, 1) + row[3] * A(3, 1),
            -(row[0] * A(0, 0) + row[1] * A(1, 0) + row[2] * A(2, 0) + row[3] * A(3, 0)),
            0.0f, 0.0f};
}

}  // namespace

GravityModel::Dof GravityModel::ModelTorque(const Dof &q_deg) const
{
    std::array<Mat4, kDof> A;
    std::array<Row, kDof + 1> row2;   // row2[k] = T_{0..k-1} 的第 3 行
    Dof tau{};
    const float g = param_.g > 0.0f ? param_.g : kDefaultG;

    /* 腰部：安装位姿 * Rz(q0)；其余关节是 DH 行 */
    A[0] = mount_ * DhTransform(DhParam{0.0f, 0.0f, 0.0f, 0.0f}, q_deg[0]);
    for (int k = 1; k < kDof; ++k)
    {
        A[k] = DhTransform(dh_[k - 1], q_deg[k]);
    }
    row2[0] = {0.0f, 0.0f, 1.0f, 0.0f};
    for (int k = 0; k < kDof; ++k)
    {
        row2[k + 1] = RowMulMat(row2[k], A[k]);
    }

    for (int j = 0; j < kDof; ++j)
    {
        Row v = RowMulDA(row2[j], A[j]);
        float sum = 0.0f;

        for (int i = j; i < kDof; ++i)
        {
            const LinkInertia &link = param_.link[i];
            const float m = link.mass_kg;

            if (i > j)
            {
                v = RowMulMat(v, A[i]);
            }
            sum += v[0] * m * link.com_m[0] + v[1] * m * link.com_m[1] +
                   v[2] * m * link.com_m[2] + v[3] * m;
        }
        tau[j] = g * sum;
    }
    return tau;
}

GravityModel::Dof GravityModel::Compute(float waist_deg, const JointArray &q_deg) const
{
    Dof q;
    Dof cmd{};
    const float sign = param_.sign < 0 ? -1.0f : 1.0f;

    if (!param_.enable)
    {
        return cmd;
    }
    q[0] = waist_deg;
    for (int i = 0; i < kJointCount; ++i)
    {
        q[i + 1] = q_deg[i];
    }
    for (float v : q)
    {
        if (!std::isfinite(v))
        {
            return cmd;
        }
    }

    const Dof model = ModelTorque(q);
    for (int i = 0; i < kDof; ++i)
    {
        const float tau = sign * model[i];

        if (((param_.joint_mask >> i) & 1U) != 0U && std::isfinite(tau))
        {
            cmd[i] = tau;
        }
    }
    return cmd;
}
