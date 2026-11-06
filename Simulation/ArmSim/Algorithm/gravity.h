#pragma once

#include <array>
#include <cstdint>

#include "kinematics.h"

/* 七轴臂重力前馈：tau_g = Y(q) * beta，纯算法，不依赖电机和物理引擎。
 * beta = [h2x h2y ... h7x h7y]，12 维，来自实机重力辨识。
 * 输出各关节保持姿态必须给的力矩 N*m。 */
class GravityModel {
public:
    static constexpr int kBetaDim = 12;
    static constexpr float kDefaultG = 9.81f;
    static constexpr uint8_t kAllJoints = (1U << kJointCount) - 1U;
    using Beta = std::array<float, kBetaDim>;

    struct Param {
        bool enable = false;
        int8_t sign = 1;                 // 前馈相对模型 dU/dq 的符号
        uint8_t joint_mask = kAllJoints; // bit i = 1：J_i 的前馈有效
        float g = kDefaultG;             // <=0 时用默认值
        Beta beta{};
    };

    GravityModel(const Param &param, const DhTable &dh) : param_(param), dh_(dh) {}

    /* 本周期的前馈力矩（已乘符号、已按 mask 屏蔽）。
     * 未使能或输入含 NaN/Inf 时全 0 */
    JointArray Compute(const JointArray &q_deg) const;

private:
    JointArray ModelTorque(const JointArray &q_deg) const;

    Param param_;
    DhTable dh_;
};
