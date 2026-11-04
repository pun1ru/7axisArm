#ifndef ARM_DYNAMICS_H
#define ARM_DYNAMICS_H

#include <stdbool.h>
#include <stdint.h>

#include "dh.h"

/* 七轴臂重力矩在线计算，tau_g = Y(q) * pi
 * 本文件只放纯算法：输入 DH 表、关节角（deg）、参数向量，输出各关节重力矩 N*m，
 * 不依赖 Arm_t / 电机 / 实时系统，与实机用的是同一份算法。
 *
 * 方向约定：输出就是 dU/dq，即关节为保持姿态必须输出的力矩；与电机指令是否同号由
 * ArmGravityParam_t.sign 决定，符号只在 ArmDynamics_GravityCommand 里乘一次。
 *
 * 参数向量有两套，数值上等价：
 *   28 维 pi  = [h1x h1y h1z m1, ... h7x h7y h7z m7]，用于核对辨识结果
 *   12 维 beta = [h2x h2y h3x h3y h4x h4y h5x h5y h6x h6y h7x h7y]，用于正式控制
 */
#define ARM_GRAVITY_DOF         7U
#define ARM_GRAVITY_PI_DIM      (4U * ARM_GRAVITY_DOF)
#define ARM_GRAVITY_BETA_DIM    12U
#define ARM_GRAVITY_DEFAULT_G   9.81f
#define ARM_GRAVITY_ALL_JOINTS  ((1U << ARM_GRAVITY_DOF) - 1U)

/* beta[k] 对应 pi 的哪一列（0 基），顺序即 beta 的顺序 */
#define ARM_GRAVITY_BETA_SEL    {4U, 5U, 8U, 9U, 12U, 13U, 16U, 17U, 20U, 21U, 24U, 25U}

typedef struct {
    uint8_t enable;     /* 0：不计算，重力矩恒为 0 */
    uint8_t use_beta;   /* 1：用 12 维 beta；0：用 28 维 pi */
    int8_t sign;        /* 重力前馈相对模型 dU/dq 的符号：+1 同号；-1 反号 */
    uint8_t joint_mask; /* bit i = 1：J_i 的前馈指令有效 */
    float g;            /* 重力加速度 m/s^2，<=0 时用默认值 */
    const float *pi;    /* ARM_GRAVITY_PI_DIM 个，可为 NULL */
    const float *beta;  /* ARM_GRAVITY_BETA_DIM 个，可为 NULL */
} ArmGravityParam_t;

/**
 * \brief 28 维 pi 版本
 * \note  中间量放在文件内 static 工作区，不可重入；仿真里只有控制任务调用。
 */
void ArmDynamics_GravityTorquePi(const DH_Param_t *dh, const float *q_deg, const float *pi,
                                 float g, float *tau);

/** \brief 12 维 beta 版本，参数含义、约束同 ArmDynamics_GravityTorquePi */
void ArmDynamics_GravityTorqueBeta(const DH_Param_t *dh, const float *q_deg, const float *beta,
                                   float g, float *tau);

/**
 * \brief 按参数结构选 pi / beta 计算原始重力矩（不乘符号、不限幅）
 * \return true 正常；false：未使能 / 参数表为空 / 输入含 NaN 或 Inf，此时 tau 已清零
 */
bool ArmDynamics_GravityModel(const ArmGravityParam_t *param, const DH_Param_t *dh,
                              const float *q_deg, float *tau);

/**
 * \brief 模型力矩 -> 前馈指令：乘符号（只在这里乘一次），再按各关节力矩限幅截断
 * \param[in]  limit  各关节独立限幅 N*m；NULL 表示不做单轴预限幅
 * \param[in]  mask   joint_mask：bit i = 0 的关节输出 0
 */
void ArmDynamics_GravityCommand(const float *model, bool valid, int8_t sign, const float *limit,
                                uint8_t mask, float *cmd);

#endif
