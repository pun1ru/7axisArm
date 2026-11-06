#pragma once

#include <array>
#include <cstdint>

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

/* URDF 的 <origin xyz rpy>：R = Rz(yaw) * Ry(pitch) * Rx(roll)，rpy 单位 rad */
Mat4 TransformFromXyzRpy(const std::array<float, 3> &xyz_m, const std::array<float, 3> &rpy_rad);

Mat4 DhTransform(const DhParam &param, float q_deg);
/* 第 k 个元素 = 坐标系 k+1 相对基座的位姿 T_{0,k+1}（k = 0..6），最后一个就是末端 */
using FrameArray = std::array<Mat4, kJointCount>;
FrameArray ForwardKinematicsFrames(const DhTable &table, const JointArray &q_deg);
Mat4 ForwardKinematics(const DhTable &table, const JointArray &q_deg);
/* pitch 范围 [-90, 90]；万向节死锁时约定 yaw = 0，转角全部归到 roll */
Pose PoseFromTransform(const Mat4 &T);

/*--------------------------------------S-R-S 构型：臂角与分支--------------------------------------*
 * 前 3 轴交于肩点 S（坐标系 1/2/3 的 z 轴共点），第 4 轴是肘，后 3 轴交于腕点 W（坐标系 5/6/7 的 z 轴共点）。
 *   S = 坐标系 1 的原点，E = 坐标系 3 的原点（上臂轴与肘轴的交点），W = 坐标系 5 的原点。
 * 下面的臂角和分支都建立在这个结构上，DH 表不满足时不要用。
 * 注意编号：这里的 θ2 / θ4 / θ6 是主臂第 2 / 4 / 6 个关节（q_deg 下标 1 / 3 / 5，URDF 里叫 joint3 / joint5 / joint7）。 */
bool IsSrsStructure(const DhTable &table);

/* 臂角 ψ 属于逆解的参数化，定义和计算放在 ik.h（ComputeArmAngle）。 */

/* 全局构型分支 λ2、λ4、λ6（论文式 3-3），各取 ±1，共 2³ = 8 个分支。
 * λk = sign(sin θk)，θk 是 DH 角（关节角 + theta_offset），sin = 0 时取 +1。
 *   肩 λ2：θ2 = 0 或 180° 时上臂轴与第 1 轴重合（肩部欧拉角奇异）
 *   肘 λ4：θ4 = 0 或 180° 时前臂与上臂共线（臂伸直或折叠）
 *   腕 λ6：θ6 = 0 或 180° 时第 5、7 轴重合（腕部欧拉角奇异）
 * 分支之间只能经过这些奇异位置切换，所以连续运动中分支不会变。 */
struct Branch {
    bool valid = false;
    int8_t shoulder = 1;
    int8_t elbow = 1;
    int8_t wrist = 1;

    /* 0..7，bit0 = 肩、bit1 = 肘、bit2 = 腕，对应 λ = -1 时置位；(+,+,+) = 0 */
    uint8_t index() const
    {
        return static_cast<uint8_t>((shoulder < 0 ? 1U : 0U) | (elbow < 0 ? 2U : 0U) | (wrist < 0 ? 4U : 0U));
    }
};
Branch ComputeBranch(const DhTable &table, const JointArray &q_deg);
