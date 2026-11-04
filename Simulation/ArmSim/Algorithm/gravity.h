#ifndef ARM_DYNAMICS_H
#define ARM_DYNAMICS_H

#include <stdbool.h>
#include <stdint.h>

#include "dh.h"

/* 七轴臂重力矩在线计算，tau_g = Y(q) * pi，纯算法，不依赖 Arm_t / 电机 / 实时系统。
 * 输入 DH 表、关节角 deg、参数向量，输出各关节重力矩 N*m（dU/dq，即保持姿态必须输出的力矩）。
 * 与电机指令是否同号由 ArmGravityParam_t.sign 决定，只在 ArmDynamics_GravityCommand 里乘一次。
 *
 * 参数向量两套，数值等价：
 *   28 维 pi   = [h1x h1y h1z m1, ... h7x h7y h7z m7]，用于核对辨识结果
 *   12 维 beta = [h2x h2y ... h7x h7y]，用于正式控制 */
#define ARM_GRAVITY_DOF         7U
#define ARM_GRAVITY_PI_DIM      (4U * ARM_GRAVITY_DOF)
#define ARM_GRAVITY_BETA_DIM    12U
#define ARM_GRAVITY_DEFAULT_G   9.81f
#define ARM_GRAVITY_ALL_JOINTS  ((1U << ARM_GRAVITY_DOF) - 1U)

/* beta[k] 对应 pi 的列号（0 基） */
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

/* 28 维 pi 版本。中间量用文件内 static 工作区，不可重入 */
void ArmDynamics_GravityTorquePi(const DH_Param_t *dh, const float *q_deg, const float *pi,
                                 float g, float *tau);

/* 12 维 beta 版本，约束同上 */
void ArmDynamics_GravityTorqueBeta(const DH_Param_t *dh, const float *q_deg, const float *beta,
                                   float g, float *tau);

/* 按参数选 pi / beta 算原始重力矩（不乘符号、不限幅）。
 * 返回 false：未使能 / 参数表为空 / 输入含 NaN，此时 tau 已清零 */
bool ArmDynamics_GravityModel(const ArmGravityParam_t *param, const DH_Param_t *dh,
                              const float *q_deg, float *tau);

/* 模型力矩 -> 前馈指令：乘符号，按 limit（N*m，NULL 不限）截断，mask 为 0 的关节输出 0 */
void ArmDynamics_GravityCommand(const float *model, bool valid, int8_t sign, const float *limit,
                                uint8_t mask, float *cmd);

#endif
