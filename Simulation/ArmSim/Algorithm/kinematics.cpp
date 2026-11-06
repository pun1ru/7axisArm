#include "kinematics.h"

#include <cmath>

namespace {
constexpr float kGimbalLockEps = 1e-6f;   // cos(pitch) 低于此值视为万向节死锁

bool IsZero(float v) { return std::fabs(v) < 1e-6f; }
bool IsRightAngle(float deg) { return std::fabs(std::fabs(deg) - 90.0f) < 1e-3f; }
}  // namespace

Mat4 Mat4::Identity()
{
    Mat4 T;
    for (int i = 0; i < 4; ++i)
    {
        T.m[i][i] = 1.0f;
    }
    return T;
}

Mat4 Mat4::operator*(const Mat4 &B) const
{
    const Mat4 &A = *this;
    Mat4 out;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out(i, j) = A(i, 0) * B(0, j) + A(i, 1) * B(1, j) + A(i, 2) * B(2, j);
        }
        out(i, 3) = A(i, 0) * B(0, 3) + A(i, 1) * B(1, 3) + A(i, 2) * B(2, 3) + A(i, 3);
    }
    out(3, 3) = 1.0f;
    return out;
}

Mat4 TransformFromXyzRpy(const std::array<float, 3> &xyz_m, const std::array<float, 3> &rpy_rad)
{
    const float cr = std::cos(rpy_rad[0]), sr = std::sin(rpy_rad[0]);
    const float cp = std::cos(rpy_rad[1]), sp = std::sin(rpy_rad[1]);
    const float cy = std::cos(rpy_rad[2]), sy = std::sin(rpy_rad[2]);
    Mat4 T;

    T.m = {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz_m[0]},
            {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz_m[1]},
            {-sp,     cp * sr,                cp * cr,                xyz_m[2]},
            {0.0f,    0.0f,                   0.0f,                   1.0f}}};
    return T;
}

Mat4 DhTransform(const DhParam &p, float q_deg)
{
    const float theta = (q_deg + p.theta_offset_deg) * kDegToRad;
    const float alpha = p.alpha_deg * kDegToRad;
    const float ct = std::cos(theta), st = std::sin(theta);
    const float ca = std::cos(alpha), sa = std::sin(alpha);
    Mat4 T;

    T.m = {{{ct,      -st,      0.0f, p.a_m},
            {st * ca, ct * ca,  -sa,  -sa * p.d_m},
            {st * sa, ct * sa,  ca,   ca * p.d_m},
            {0.0f,    0.0f,     0.0f, 1.0f}}};
    return T;
}

FrameArray ForwardKinematicsFrames(const DhTable &table, const JointArray &q_deg)
{
    FrameArray frames;
    Mat4 T = Mat4::Identity();

    for (int k = 0; k < kJointCount; ++k)
    {
        T = T * DhTransform(table[k], q_deg[k]);
        frames[k] = T;
    }
    return frames;
}

Mat4 ForwardKinematics(const DhTable &table, const JointArray &q_deg)
{
    return ForwardKinematicsFrames(table, q_deg).back();
}

Pose PoseFromTransform(const Mat4 &T)
{
    Pose pose;
    float roll, yaw;
    const float horizon = std::sqrt(T(0, 0) * T(0, 0) + T(1, 0) * T(1, 0));
    const float pitch = std::atan2(-T(2, 0), horizon);

    if (horizon > kGimbalLockEps)
    {
        roll = std::atan2(T(2, 1), T(2, 2));
        yaw = std::atan2(T(1, 0), T(0, 0));
    }
    else
    {
        roll = std::atan2(-T(1, 2), T(1, 1));
        yaw = 0.0f;
    }

    for (int i = 0; i < 3; ++i)
    {
        pose.position_m[i] = T(i, 3);
    }
    pose.euler_deg = {roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg};
    return pose;
}

/*--------------------------------------S-R-S 构型：臂角与分支--------------------------------------*/
/* DhTable 的第 k 行是坐标系 k+1 相对 k 的变换，行里的 (alpha, a) 属于上一根连杆，d 和 theta 属于本关节。
 *   肩：z1、z2、z3 交于坐标系 1 原点 <=> a1 = 0，d2 = 0，a2 = 0，且 z1⊥z2、z2⊥z3
 *   肘：a3 = a4 = 0，z3⊥z4、z4⊥z5：S→E 和 E→W' 都垂直于肘轴 z4，S-E-W' 才是平面三角形
 *       （W' 是 W 沿肘轴退回偏置 d4，见 ik.h）
 *   腕：z5、z6、z7 交于坐标系 5 原点 <=> a5 = 0，d6 = 0，a6 = 0，且 z5⊥z6、z6⊥z7 */
bool IsSrsStructure(const DhTable &t)
{
    const bool shoulder = IsZero(t[1].a_m) && IsZero(t[1].d_m) && IsZero(t[2].a_m) &&
                          IsRightAngle(t[1].alpha_deg) && IsRightAngle(t[2].alpha_deg);
    const bool elbow = IsZero(t[3].a_m) && IsZero(t[4].a_m) &&
                       IsRightAngle(t[3].alpha_deg) && IsRightAngle(t[4].alpha_deg);
    const bool wrist = IsZero(t[5].a_m) && IsZero(t[5].d_m) && IsZero(t[6].a_m) &&
                       IsRightAngle(t[5].alpha_deg) && IsRightAngle(t[6].alpha_deg);

    return shoulder && elbow && wrist;
}

Branch ComputeBranch(const DhTable &table, const JointArray &q_deg)
{
    const auto side = [&](int k) -> int8_t {
        const float theta = (q_deg[k] + table[k].theta_offset_deg) * kDegToRad;
        return std::sin(theta) >= 0.0f ? 1 : -1;
    };
    Branch b;

    b.valid = true;
    b.shoulder = side(1);   // θ2
    b.elbow = side(3);      // θ4
    b.wrist = side(5);      // θ6
    return b;
}
