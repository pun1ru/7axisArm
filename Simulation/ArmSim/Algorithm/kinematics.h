#pragma once

#include <array>

constexpr int kJointCount = 7;
constexpr float kPi = 3.141592653589793f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

using JointArray = std::array<float, kJointCount>;

/* 改进 DH（Craig），一行 = 坐标系 i 相对 i-1，角度 deg，长度 m
 *   T_{i-1,i} = Rx(alpha) * Tx(a) * Rz(q + theta_offset) * Tz(d)
 * 末端固定坐标系那一行 q 恒为 0。 */
struct DhParam {
    float alpha_deg;
    float a_m;
    float d_m;
    float theta_offset_deg;
};
using DhTable = std::array<DhParam, kJointCount>;

/* 齐次变换 [R p; 0 1]，最后一行恒为 [0 0 0 1]，乘法只算上面 3x4 */
struct Mat4 {
    std::array<std::array<float, 4>, 4> m{};

    static Mat4 Identity();
    Mat4 operator*(const Mat4 &rhs) const;
    float &operator()(int r, int c) { return m[r][c]; }
    float operator()(int r, int c) const { return m[r][c]; }
};

/* 末端位姿：位置 m，欧拉角 deg（roll, pitch, yaw） */
struct Pose {
    std::array<float, 3> position_m{};
    std::array<float, 3> euler_deg{};
};

Mat4 DhTransform(const DhParam &param, float q_deg);
Mat4 ForwardKinematics(const DhTable &table, const JointArray &q_deg);
/* pitch 范围 [-90, 90]；万向节死锁时约定 yaw = 0，转角全部归到 roll */
Pose PoseFromTransform(const Mat4 &T);
