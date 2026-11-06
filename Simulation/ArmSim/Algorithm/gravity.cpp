#include "gravity.h"

#include <cmath>

/* tau_j = g * sum_{i>=j} sz^T * T_{j-1} * dA_j * A_{j+1}...A_i * pi_i
 * 只用结果的第 3 行，而 row2(M * A) = row2(M) * A，所以只递推一个 1x4 行向量 v：
 *     v0 = row2(T_{j-1}) * dA_j,  v = v * A_i (i = j+1..6),  tau_j += g * (v . pi_i)
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

JointArray GravityModel::ModelTorque(const JointArray &q_deg) const
{
    std::array<Mat4, kJointCount> A;            // A[k] = T_{k,k+1}
    std::array<Row, kJointCount + 1> row2;      // row2[k] = T_{0..k-1} 的第 3 行
    JointArray tau{};
    const float g = param_.g > 0.0f ? param_.g : kDefaultG;

    for (int k = 0; k < kJointCount; ++k)
    {
        A[k] = DhTransform(dh_[k], q_deg[k]);
    }
    row2[0] = {0.0f, 0.0f, 1.0f, 0.0f};
    for (int k = 0; k < kJointCount; ++k)
    {
        row2[k + 1] = RowMulMat(row2[k], A[k]);
    }

    for (int j = 0; j < kJointCount; ++j)
    {
        Row v = RowMulDA(row2[j], A[j]);
        float sum = 0.0f;

        for (int i = j; i < kJointCount; ++i)
        {
            if (i > j)
            {
                v = RowMulMat(v, A[i]);
            }
            /* beta 只含连杆 1..6，每连杆 2 个：hx、hy */
            if (i >= 1)
            {
                sum += v[0] * param_.beta[2 * (i - 1)] + v[1] * param_.beta[2 * (i - 1) + 1];
            }
        }
        tau[j] = g * sum;
    }
    return tau;
}

JointArray GravityModel::Compute(const JointArray &q_deg) const
{
    JointArray cmd{};
    const float sign = param_.sign < 0 ? -1.0f : 1.0f;

    if (!param_.enable)
    {
        return cmd;
    }
    for (float q : q_deg)
    {
        if (!std::isfinite(q))
        {
            return cmd;
        }
    }

    const JointArray model = ModelTorque(q_deg);
    for (int i = 0; i < kJointCount; ++i)
    {
        const float tau = sign * model[i];

        if (((param_.joint_mask >> i) & 1U) != 0U && std::isfinite(tau))
        {
            cmd[i] = tau;
        }
    }
    return cmd;
}
