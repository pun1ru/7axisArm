#pragma once

#include <array>
#include <cstdint>

#include "kinematics.h"

/* 全链重力前馈：腰部 1 个 + 机械臂 7 个共 8 个自由度。
 * 势能 U = g * sum_i m_i * z(COM_i)，保持姿态必须给的力矩 tau_j = dU/dq_j。
 * 每个连杆只需要质量和质心（连杆自身坐标系下），纯算法，不依赖电机和物理引擎。
 *
 * 链的结构：world --mount--> Rz(q_waist) --DH_0--> ... --DH_6--> 末端
 * 连杆 0 是随腰部转动的 link1，连杆 1..7 是机械臂 link2..link8（夹爪并入最后一个连杆）。 */
class GravityModel {
public:
    static constexpr int kDof = kJointCount + 1;
    static constexpr float kDefaultG = 9.81f;
    static constexpr uint8_t kAllJoints = (1U << kDof) - 1U;
    using Dof = std::array<float, kDof>;

    struct LinkInertia {
        float mass_kg = 0.0f;
        std::array<float, 3> com_m{};   // 质心，连杆自身坐标系
    };

    struct Param {
        bool enable = false;
        int8_t sign = 1;                 // 前馈相对模型 dU/dq 的符号
        uint8_t joint_mask = kAllJoints; // bit 0 = 腰部，bit i = 机械臂第 i-1 关节
        float g = kDefaultG;             // <=0 时用默认值
        std::array<LinkInertia, kDof> link{};
    };

    /* mount：腰部关节坐标系相对世界的安装位姿（腰部转角为 0 时） */
    GravityModel(const Param &param, const Mat4 &mount, const DhTable &dh)
        : param_(param), mount_(mount), dh_(dh) {}

    /* 本周期的前馈力矩（已乘符号、已按 mask 屏蔽）。未使能或输入含 NaN/Inf 时全 0 */
    Dof Compute(float waist_deg, const JointArray &q_deg) const;

private:
    Dof ModelTorque(const std::array<float, kDof> &q_deg) const;

    Param param_;
    Mat4 mount_;
    DhTable dh_;
};
