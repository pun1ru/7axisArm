#ifndef ARM_H
#define ARM_H

#include <stdbool.h>
#include <stdint.h>

#include "dh.h"
#include "gravity.h"
#include "motor.h"
#include "pid.h"

/* Module 层：七轴臂对象，对象划分与 Real/DIYcontroller/LibSE/Module/Arm 相同。
 *   配置三块：电机 / 状态 / 参数，具体数值（DH、限位、增益、重力）由 System 层给出
 *   每周期 Arm_Run：观测 -> 控制 -> 通信
 * 仿真没有 CAN 总线，电机在 Driver 层已经和 SimPlant 绑定，这里只管算法和状态机。 */
#define ARM_JOINT_COUNT   7U
#define ARM_VECTOR_DIM    3U

/*--------------------------------------状态机--------------------------------------*/
typedef enum {
    ARM_ROLE_MASTER = 0,
    ARM_ROLE_SERVANT
} ArmRole_t;

typedef enum {
    ARM_PROTECTED = 0,  /* 保护：每个电机发零指令 */
    ARM_ENABLE,         /* 使能：全部关节闭环，目标对齐当前关节角 */
    ARM_FEEDBACK,       /* 主机：只读反馈 */
    ARM_MASTER,         /* 主机：正常工作 */
    ARM_AUTO,           /* 从机：自主运行 */
    ARM_SERVANT,        /* 从机：跟随主机 */
    ARM_HOST            /* 上位机设定目标，是否闭环由上位机决定 */
} ArmState_t;

typedef enum {
    ARM_HOST_TARGET_OK = 0,
    ARM_HOST_TARGET_WRONG_STATE,
    ARM_HOST_TARGET_BAD_TARGET
} ArmHostTargetResult_t;

typedef enum {
    ARM_J0 = 0,
    ARM_J1,
    ARM_J2,
    ARM_J3,
    ARM_J4,
    ARM_J5,
    ARM_J6
} ArmJoint_t;

/*--------------------------------------配置--------------------------------------*/
typedef struct {
    Motor_t *joint[ARM_JOINT_COUNT];
} ArmMotor_t;

typedef struct {
    ArmRole_t role;
    ArmState_t init_state;
} ArmStateInit_t;

typedef struct {
    float min_deg;
    float max_deg;
} ArmJointLimit_t;

/* 误差 deg -> 力矩 N*m；D 项按每周期误差增量计算（周期并入 kd）
 * total_output_max 限制 PID + 重力前馈的最终总和，<=0 时该关节输出恒为 0 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float d_alpha;
    float sum_error_max;
    float output_max;
    float total_output_max;
} ArmJointGain_t;

typedef struct {
    DH_Param_t dh[ARM_JOINT_COUNT];
    float zero_point_rad[ARM_JOINT_COUNT];
    int8_t direction[ARM_JOINT_COUNT];
    ArmJointLimit_t limit[ARM_JOINT_COUNT];
    ArmJointGain_t gain[ARM_JOINT_COUNT];
    ArmGravityParam_t gravity;
} ArmParam_t;

typedef struct {
    ArmMotor_t motor;
    ArmStateInit_t state;
    ArmParam_t param;
} ArmConfig_t;

/*--------------------------------------运行状态--------------------------------------*/
typedef struct {
    float joint_deg[ARM_JOINT_COUNT];
    float joint_abs_deg[ARM_JOINT_COUNT];
    float end_to_base[4][4];
    float position_m[ARM_VECTOR_DIM];
    float euler_deg[ARM_VECTOR_DIM];
} ArmObserver_t;

typedef struct {
    PIDStruct pid;
    float target_deg;
    float limit_min_deg;
    float limit_max_deg;
    float output;
} ArmJointCtrl_t;

typedef struct {
    float torque_model[ARM_JOINT_COUNT];
    float torque_cmd[ARM_JOINT_COUNT];
    uint8_t model_valid;
} ArmGravityState_t;

typedef struct {
    ArmConfig_t config;
    ArmState_t state;
    ArmObserver_t observer;
    ArmJointCtrl_t joint_ctrl[ARM_JOINT_COUNT];
    ArmGravityState_t gravity;
    uint16_t enable_divider;
    uint8_t ctrl_align_pending;
    uint8_t host_align_pending;
    uint8_t host_closed_loop_mask;
    uint8_t host_target_valid_mask;
    float host_target_abs_deg[ARM_JOINT_COUNT];
} Arm_t;

void Arm_Init(Arm_t *arm, const ArmConfig_t *config);
void Arm_Run(Arm_t *arm);

/* 状态由上层（遥控输入 / 上位机）修改，目标状态不属于本机角色时拒绝 */
bool Arm_SetState(Arm_t *arm, ArmState_t state);

/* HOST 单关节目标：target_abs_deg 为单圈绝对角，必须落在该关节限位内 */
ArmHostTargetResult_t Arm_SetHostTarget(Arm_t *arm, uint8_t joint, float target_abs_deg,
                                        bool closed_loop);
void Arm_StopHostControl(Arm_t *arm);

/* 把当前姿态记为机械零位 */
bool Arm_CalibrateZero(Arm_t *arm);

#endif
