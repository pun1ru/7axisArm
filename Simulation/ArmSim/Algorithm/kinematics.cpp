#include "kinematics.h"

#include <cmath>

namespace {
constexpr float kGimbalLockEps = 1e-6f;   // cos(pitch) 低于此值视为万向节死锁
}

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

Mat4 ForwardKinematics(const DhTable &table, const JointArray &q_deg)
{
    Mat4 T = Mat4::Identity();

    for (int k = 0; k < kJointCount; ++k)
    {
        T = T * DhTransform(table[k], q_deg[k]);
    }
    return T;
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
