#include "arm.h"

#include <string.h>

#include "position_tran.h"

static_assert(ARM_GRAVITY_DOF == ARM_JOINT_COUNT, "gravity.h 的关节数必须等于 ARM_JOINT_COUNT");

#define ARM_ENABLE_RESEND_DIV   200U

#define ARM_STATE_BIT(state)    (1U << (state))
#define ARM_MASTER_STATE_MASK   (ARM_STATE_BIT(ARM_PROTECTED) | ARM_STATE_BIT(ARM_ENABLE) | \
                                 ARM_STATE_BIT(ARM_FEEDBACK) | ARM_STATE_BIT(ARM_MASTER) | \
                                 ARM_STATE_BIT(ARM_HOST))
#define ARM_SERVANT_STATE_MASK  (ARM_STATE_BIT(ARM_PROTECTED) | ARM_STATE_BIT(ARM_ENABLE) | \
                                 ARM_STATE_BIT(ARM_AUTO) | ARM_STATE_BIT(ARM_SERVANT) | \
                                 ARM_STATE_BIT(ARM_HOST))

static void Arm_SendZeroCommand(Arm_t *arm);
static void Arm_SendJointCommand(Arm_t *arm);

/*--------------------------------------工具--------------------------------------*/
static float DoubleEdgeLimiter(float value, float min_limit, float max_limit)
{
    if (value > max_limit)
    {
        return max_limit;
    }
    if (value < min_limit)
    {
        return min_limit;
    }
    return value;
}

static bool Arm_StateAllowed(const Arm_t *arm, ArmState_t state)
{
    const uint32_t mask = arm->config.state.role == ARM_ROLE_MASTER ? ARM_MASTER_STATE_MASK :
                                                                     ARM_SERVANT_STATE_MASK;

    return (mask & ARM_STATE_BIT(state)) != 0U;
}

/*--------------------------------------观测--------------------------------------*/
/* 电机反馈 -> 关节角，方向按 direction 折算（+1 正向转编码器增大） */
static void Arm_UpdateJointAngle(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];
        const int8_t sign = arm->config.param.direction[i];
        const float direction = sign < 0 ? -1.0f : 1.0f;

        Motor_UpdateFeedbackStatus(motor);
        arm->observer.joint_abs_deg[i] = direction * motor->feedback.abs_angle_deg;
        arm->observer.joint_deg[i] = direction * motor->feedback.multi_angle_deg;
    }
}

static void Arm_UpdateForwardKinematics(Arm_t *arm)
{
    DH_ForwardKinematics(arm->config.param.dh, arm->observer.joint_deg, ARM_JOINT_COUNT,
                         arm->observer.end_to_base);
}

static void Arm_ExtractPose(Arm_t *arm)
{
    ArmObserver_t *obs = &arm->observer;
    float rotation[3][3];
    float euler_rad[ARM_VECTOR_DIM];

    Trans4_GetPos(obs->end_to_base, obs->position_m);
    Trans4_GetRotation(obs->end_to_base, rotation);
    Matrix_ToEuler(rotation, euler_rad);
    for (uint8_t i = 0U; i < ARM_VECTOR_DIM; ++i)
    {
        obs->euler_deg[i] = euler_rad[i] * MOTOR_RAD_TO_DEG;
    }
}

/*--------------------------------------重力补偿--------------------------------------*/
static void Arm_GravityClear(Arm_t *arm)
{
    memset(arm->gravity.torque_cmd, 0, sizeof(arm->gravity.torque_cmd));
    arm->gravity.output_active = 0U;
}

static void Arm_GravityUpdate(Arm_t *arm)
{
    ArmGravityState_t *gravity = &arm->gravity;

    gravity->model_valid = ArmDynamics_GravityModel(&arm->config.param.gravity, arm->config.param.dh,
                                                    arm->observer.joint_deg, gravity->torque_model) ? 1U : 0U;
    gravity->output_active = 0U;
    /* 符号只在这里乘一次；不做单轴预限幅，PID + 前馈相加后统一做总限幅 */
    ArmDynamics_GravityCommand(gravity->torque_model, gravity->model_valid != 0U,
                               arm->config.param.gravity.sign, NULL,
                               arm->config.param.gravity.joint_mask, gravity->torque_cmd);
}

/*--------------------------------------目标--------------------------------------*/
static void Arm_UpdateMultiTurnLimit(const Arm_t *arm, uint8_t joint, ArmJointCtrl_t *ctrl)
{
    const ArmJointLimit_t *limit = &arm->config.param.limit[joint];
    const float current_abs_deg = arm->observer.joint_abs_deg[joint];
    const float current_multi_deg = arm->observer.joint_deg[joint];

    /* 限位映射到当前圈，差值不做环绕 */
    ctrl->limit_min_deg = current_multi_deg + (limit->min_deg - current_abs_deg);
    ctrl->limit_max_deg = current_multi_deg + (limit->max_deg - current_abs_deg);
}

static void Arm_AlignJointTarget(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];

        Arm_UpdateMultiTurnLimit(arm, i, ctrl);
        ctrl->target_deg = DoubleEdgeLimiter(arm->observer.joint_deg[i],
                                             ctrl->limit_min_deg, ctrl->limit_max_deg);
        ctrl->output = 0.0f;
        PIDReset(&ctrl->pid);
    }
}

static void Arm_AlignHostTarget(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];

        Arm_UpdateMultiTurnLimit(arm, i, ctrl);
        ctrl->target_deg = DoubleEdgeLimiter(arm->observer.joint_deg[i],
                                             ctrl->limit_min_deg, ctrl->limit_max_deg);
        ctrl->output = 0.0f;
        PIDReset(&ctrl->pid);
        arm->host_target_abs_deg[i] = arm->observer.joint_abs_deg[i];
    }
    arm->host_closed_loop_mask = 0U;
    arm->host_target_valid_mask = 0U;
}

static void Arm_TrackJointTarget(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        arm->joint_ctrl[i].target_deg = arm->observer.joint_deg[i];
    }
}

static void Arm_UpdateTarget(Arm_t *arm)
{
    switch (arm->state)
    {
    case ARM_ENABLE:
        if (arm->ctrl_align_pending != 0U)
        {
            Arm_AlignJointTarget(arm);
            arm->ctrl_align_pending = 0U;
        }
        for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
        {
            ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];

            ctrl->target_deg = DoubleEdgeLimiter(arm->observer.joint_deg[i],
                                                 ctrl->limit_min_deg, ctrl->limit_max_deg);
        }
        break;

    case ARM_HOST:
        if (arm->host_align_pending != 0U)
        {
            Arm_AlignHostTarget(arm);
            arm->host_align_pending = 0U;
            arm->ctrl_align_pending = 0U;
        }
        for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
        {
            ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];

            ctrl->target_deg = DoubleEdgeLimiter(ctrl->target_deg,
                                                 ctrl->limit_min_deg, ctrl->limit_max_deg);
        }
        break;

    default:
        Arm_TrackJointTarget(arm);
        break;
    }
}

/*--------------------------------------控制--------------------------------------*/
/* 总力矩限幅：PID 与重力前馈各自限幅后相加，最终和再统一截到这里 */
static float Arm_LimitTotalTorque(const Arm_t *arm, uint8_t joint, float torque)
{
    const float limit = arm->config.param.gain[joint].total_output_max;

    if (!(limit > 0.0f) || !((torque - torque) == 0.0f))
    {
        return 0.0f;
    }
    return torque > limit ? limit : (torque < -limit ? -limit : torque);
}

static float Arm_AddGravity(Arm_t *arm, uint8_t joint, float pid_output)
{
    if (arm->config.param.gravity.enable == 0U || arm->gravity.model_valid == 0U)
    {
        return pid_output;
    }
    arm->gravity.output_active = 1U;
    return pid_output + arm->gravity.torque_cmd[joint];
}

static void Arm_JointControl(Arm_t *arm)
{
    if (arm->state != ARM_ENABLE && arm->state != ARM_HOST)
    {
        Arm_SendZeroCommand(arm);
        return;
    }

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];
        const float observed_deg = arm->observer.joint_deg[i];
        const bool closed_loop =
            (arm->state == ARM_ENABLE) ||
            ((arm->host_closed_loop_mask & (uint8_t)(1U << i)) != 0U);

        if (closed_loop)
        {
            /* 误差直接相减，不做最短路径折叠，理由同实机 arm_jointctrl.c */
            ctrl->output = PIDUpdate(&ctrl->pid, ctrl->target_deg - observed_deg);
        }
        else
        {
            ctrl->output = 0.0f;
            PIDReset(&ctrl->pid);
        }
        ctrl->output = Arm_AddGravity(arm, i, ctrl->output);
        ctrl->output = Arm_LimitTotalTorque(arm, i, ctrl->output);
    }
    Arm_SendJointCommand(arm);
}

/*--------------------------------------通信--------------------------------------*/
static void Arm_WriteJointCommand(Motor_t *motor, float torque_nm)
{
    if (motor == NULL)
    {
        return;
    }

    MotorCmd_t *cmd = &motor->motorcmd;
    switch (cmd->control_mode)
    {
    case MODE_MIT:
        cmd->position = 0.0f;
        cmd->velocity = 0.0f;
        cmd->kp = 0.0f;
        cmd->kd = 0.0f;
        cmd->torque = torque_nm;
        break;
    case MODE_TORQUE:
        cmd->torque = torque_nm;
        break;
    case MODE_CURRENT:
        cmd->current = torque_nm;
        break;
    default:
        break;
    }
}

static void Arm_SendZeroCommand(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];
        const ControlMode_t mode = motor->motorcmd.control_mode;

        memset(&motor->motorcmd, 0, sizeof(motor->motorcmd));
        motor->motorcmd.control_mode = mode;
        motor->motorops.sendcommand(motor);
    }
}

static void Arm_SendJointCommand(Arm_t *arm)
{
    /* 先写完所有关节再统一发送 */
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Arm_WriteJointCommand(arm->config.motor.joint[i], arm->joint_ctrl[i].output);
    }
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];

        if (motor != NULL)
        {
            motor->motorops.sendcommand(motor);
        }
    }
}

static void Arm_SendEnableCommand(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];

        if (motor != NULL && motor->motorops.enable != NULL)
        {
            motor->motorops.enable(motor);
        }
    }
}

static void Arm_SendDisableCommand(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];

        if (motor != NULL && motor->motorops.disable != NULL)
        {
            motor->motorops.disable(motor);
        }
    }
}

/*--------------------------------------对外接口--------------------------------------*/
void Arm_Init(Arm_t *arm, const ArmConfig_t *config)
{
    memset(arm, 0, sizeof(*arm));
    arm->config = *config;
    arm->state = config->state.init_state;
    arm->ctrl_align_pending = 1U;
    arm->host_align_pending = config->state.init_state == ARM_HOST ? 1U : 0U;

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = config->motor.joint[i];
        const ArmJointGain_t *gain = &config->param.gain[i];

        /* 零点以模块参数为准，覆盖电机初始化时的默认值 */
        motor->zero_point_rad = config->param.zero_point_rad[i];
        PIDInitialize(&arm->joint_ctrl[i].pid, gain->kp, gain->ki, gain->kd,
                      gain->d_alpha, gain->sum_error_max, gain->output_max);
    }
}

void Arm_Run(Arm_t *arm)
{
    /* 观测 */
    Arm_UpdateJointAngle(arm);
    Arm_UpdateForwardKinematics(arm);
    Arm_ExtractPose(arm);
    /* 重力矩依赖本周期关节角，放在观测之后、控制之前 */
    Arm_GravityUpdate(arm);

    /* 控制与通信 */
    if (arm->state == ARM_ENABLE || arm->state == ARM_HOST)
    {
        if (arm->enable_divider == 0U)
        {
            Arm_SendEnableCommand(arm);
            arm->enable_divider = ARM_ENABLE_RESEND_DIV;
        }
        --arm->enable_divider;
    }

    Arm_UpdateTarget(arm);
    Arm_JointControl(arm);
}

bool Arm_SetState(Arm_t *arm, ArmState_t state)
{
    const ArmState_t previous_state = arm->state;

    if (!Arm_StateAllowed(arm, state))
    {
        return false;
    }

    if (state != arm->state)
    {
        arm->state = state;
        arm->enable_divider = 0U;
        arm->ctrl_align_pending = 1U;
        arm->host_align_pending = state == ARM_HOST ? 1U : 0U;
        arm->host_closed_loop_mask = 0U;
        arm->host_target_valid_mask = 0U;

        if (previous_state != ARM_PROTECTED && state == ARM_PROTECTED)
        {
            Arm_SendDisableCommand(arm);
        }
    }
    return true;
}

void Arm_SetRcOnline(Arm_t *arm, bool online)
{
    if (arm != NULL)
    {
        arm->rc_online = online ? 1U : 0U;
    }
}

ArmHostTargetResult_t Arm_SetHostTarget(Arm_t *arm, uint8_t joint, float target_abs_deg,
                                        bool closed_loop)
{
    const ArmJointLimit_t *limit;
    uint8_t bit;
    bool was_closed_loop;

    if (arm == NULL || joint >= ARM_JOINT_COUNT)
    {
        return ARM_HOST_TARGET_BAD_TARGET;
    }
    if (arm->state != ARM_HOST)
    {
        return ARM_HOST_TARGET_WRONG_STATE;
    }

    limit = &arm->config.param.limit[joint];
    if (!(target_abs_deg >= limit->min_deg && target_abs_deg <= limit->max_deg))
    {
        return ARM_HOST_TARGET_BAD_TARGET;
    }

    /* 第一次 HOST 控制前可能先收到目标：先锁存限位，再写目标 */
    if (arm->host_align_pending != 0U)
    {
        Arm_AlignHostTarget(arm);
        arm->host_align_pending = 0U;
        arm->ctrl_align_pending = 0U;
    }

    bit = (uint8_t)(1U << joint);
    was_closed_loop = (arm->host_closed_loop_mask & bit) != 0U;
    arm->host_target_abs_deg[joint] = target_abs_deg;
    arm->joint_ctrl[joint].target_deg = arm->observer.joint_deg[joint] +
                                        (target_abs_deg - arm->observer.joint_abs_deg[joint]);
    arm->host_target_valid_mask |= bit;
    if (closed_loop)
    {
        arm->host_closed_loop_mask |= bit;
    }
    else
    {
        arm->host_closed_loop_mask &= (uint8_t)~bit;
    }
    if (closed_loop != was_closed_loop)
    {
        PIDReset(&arm->joint_ctrl[joint].pid);
    }
    return ARM_HOST_TARGET_OK;
}

void Arm_StopHostControl(Arm_t *arm)
{
    if (arm == NULL)
    {
        return;
    }

    arm->host_closed_loop_mask = 0U;
    Arm_GravityClear(arm);
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        arm->joint_ctrl[i].output = 0.0f;
        PIDReset(&arm->joint_ctrl[i].pid);
    }
}

bool Arm_CalibrateZero(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        if (arm->config.motor.joint[i]->feedback.status == MOTOR_NO_FEEDBACK)
        {
            return false;
        }
    }

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];

        /* 当前原始角 = multi_angle_rad + zero_point_rad；把零点挪到当前姿态 */
        motor->zero_point_rad += motor->feedback.multi_angle_rad;
        arm->config.param.zero_point_rad[i] = motor->zero_point_rad;
    }
    return true;
}
