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

/*--------------------------------------工具--------------------------------------*/
static float Clamp(float value, float min_limit, float max_limit)
{
    return value > max_limit ? max_limit : (value < min_limit ? min_limit : value);
}

static bool Arm_StateAllowed(const Arm_t *arm, ArmState_t state)
{
    const uint32_t mask = arm->config.state.role == ARM_ROLE_MASTER ? ARM_MASTER_STATE_MASK :
                                                                     ARM_SERVANT_STATE_MASK;

    return (mask & ARM_STATE_BIT(state)) != 0U;
}

/* 对所有电机调用同一个 ops（enable / disable） */
static void Arm_MotorsCall(Arm_t *arm, void (*MotorOps_t::*op)(Motor_t *))
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];

        if ((motor->motorops.*op) != NULL)
        {
            (motor->motorops.*op)(motor);
        }
    }
}

/*--------------------------------------观测--------------------------------------*/
/* 电机反馈 -> 关节角（按 direction 折算）-> 正运动学 -> 末端位置和欧拉角 */
static void Arm_Observe(Arm_t *arm)
{
    ArmObserver_t *obs = &arm->observer;
    float rotation[3][3];
    float euler_rad[ARM_VECTOR_DIM];

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        const Feedback_t *fb = &arm->config.motor.joint[i]->feedback;
        const float direction = arm->config.param.direction[i] < 0 ? -1.0f : 1.0f;

        obs->joint_abs_deg[i] = direction * fb->abs_angle_deg;
        obs->joint_deg[i] = direction * fb->multi_angle_deg;
    }

    DH_ForwardKinematics(arm->config.param.dh, obs->joint_deg, ARM_JOINT_COUNT, obs->end_to_base);
    Trans4_GetPos(obs->end_to_base, obs->position_m);
    Trans4_GetRotation(obs->end_to_base, rotation);
    Matrix_ToEuler(rotation, euler_rad);
    for (uint8_t i = 0U; i < ARM_VECTOR_DIM; ++i)
    {
        obs->euler_deg[i] = euler_rad[i] * MOTOR_RAD_TO_DEG;
    }
}

/* 重力矩依赖本周期关节角，放在观测之后、控制之前。
 * 符号只在这里乘一次；不做单轴预限幅，PID + 前馈相加后统一做总限幅 */
static void Arm_GravityUpdate(Arm_t *arm)
{
    const ArmGravityParam_t *param = &arm->config.param.gravity;
    ArmGravityState_t *gravity = &arm->gravity;

    gravity->model_valid = ArmDynamics_GravityModel(param, arm->config.param.dh,
                                                    arm->observer.joint_deg, gravity->torque_model);
    ArmDynamics_GravityCommand(gravity->torque_model, gravity->model_valid != 0U, param->sign,
                               NULL, param->joint_mask, gravity->torque_cmd);
}

/*--------------------------------------目标--------------------------------------*/
/* 限位映射到当前圈（差值不做环绕），目标对齐当前角，PID 清零 */
static void Arm_AlignTarget(Arm_t *arm)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];
        const ArmJointLimit_t *limit = &arm->config.param.limit[i];
        const float multi_deg = arm->observer.joint_deg[i];
        const float abs_deg = arm->observer.joint_abs_deg[i];

        ctrl->limit_min_deg = multi_deg + (limit->min_deg - abs_deg);
        ctrl->limit_max_deg = multi_deg + (limit->max_deg - abs_deg);
        ctrl->target_deg = Clamp(multi_deg, ctrl->limit_min_deg, ctrl->limit_max_deg);
        ctrl->output = 0.0f;
        PIDReset(&ctrl->pid);
    }
    arm->ctrl_align_pending = 0U;
}

/* HOST 的对齐：在 AlignTarget 之上再清掉上位机的目标和闭环掩码 */
static void Arm_AlignHost(Arm_t *arm)
{
    Arm_AlignTarget(arm);
    memcpy(arm->host_target_abs_deg, arm->observer.joint_abs_deg, sizeof(arm->host_target_abs_deg));
    arm->host_closed_loop_mask = 0U;
    arm->host_target_valid_mask = 0U;
    arm->host_align_pending = 0U;
}

static void Arm_UpdateTarget(Arm_t *arm)
{
    if (arm->state == ARM_HOST && arm->host_align_pending != 0U)
    {
        Arm_AlignHost(arm);
    }
    else if (arm->state == ARM_ENABLE && arm->ctrl_align_pending != 0U)
    {
        Arm_AlignTarget(arm);
    }

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];

        switch (arm->state)
        {
        case ARM_ENABLE:   /* 目标每周期跟随观测，靠重力前馈托住 */
            ctrl->target_deg = Clamp(arm->observer.joint_deg[i], ctrl->limit_min_deg, ctrl->limit_max_deg);
            break;
        case ARM_HOST:     /* 目标由 Arm_SetHostTarget 写入，这里只做限位 */
            ctrl->target_deg = Clamp(ctrl->target_deg, ctrl->limit_min_deg, ctrl->limit_max_deg);
            break;
        default:
            ctrl->target_deg = arm->observer.joint_deg[i];
            break;
        }
    }
}

/*--------------------------------------控制与通信--------------------------------------*/
/* 把各关节力矩写进 motorcmd 并发送；active=false 时发零指令（保留控制模式） */
static void Arm_SendTorque(Arm_t *arm, bool active)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        Motor_t *motor = arm->config.motor.joint[i];
        MotorCmd_t *cmd = &motor->motorcmd;
        const ControlMode_t mode = cmd->control_mode;

        memset(cmd, 0, sizeof(*cmd));
        cmd->control_mode = mode;
        if (active && mode == MODE_CURRENT)
        {
            cmd->current = arm->joint_ctrl[i].output;
        }
        else if (active)
        {
            cmd->torque = arm->joint_ctrl[i].output;
        }
        motor->motorops.sendcommand(motor);
    }
}

/* PID（ENABLE 全关节，HOST 只对闭环掩码里的关节）+ 重力前馈，再统一做总力矩限幅 */
static void Arm_JointControl(Arm_t *arm)
{
    if (arm->state != ARM_ENABLE && arm->state != ARM_HOST)
    {
        Arm_SendTorque(arm, false);
        return;
    }

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        ArmJointCtrl_t *ctrl = &arm->joint_ctrl[i];
        const float limit = arm->config.param.gain[i].total_output_max;
        const bool closed_loop = arm->state == ARM_ENABLE ||
                                 (arm->host_closed_loop_mask & (1U << i)) != 0U;
        float torque = 0.0f;

        if (closed_loop)
        {
            /* 误差直接相减，不做最短路径折叠，理由同实机 arm_jointctrl.c */
            torque = PIDUpdate(&ctrl->pid, ctrl->target_deg - arm->observer.joint_deg[i]);
        }
        else
        {
            PIDReset(&ctrl->pid);
        }
        if (arm->config.param.gravity.enable != 0U && arm->gravity.model_valid != 0U)
        {
            torque += arm->gravity.torque_cmd[i];
        }
        /* limit<=0 或结果非有限时输出 0 */
        ctrl->output = (limit > 0.0f && (torque - torque) == 0.0f) ? Clamp(torque, -limit, limit) : 0.0f;
    }
    Arm_SendTorque(arm, true);
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
        const ArmJointGain_t *gain = &config->param.gain[i];

        /* 零点以模块参数为准，覆盖电机初始化时的默认值 */
        config->motor.joint[i]->zero_point_rad = config->param.zero_point_rad[i];
        PIDInitialize(&arm->joint_ctrl[i].pid, gain->kp, gain->ki, gain->kd,
                      gain->d_alpha, gain->sum_error_max, gain->output_max);
    }
}

void Arm_Run(Arm_t *arm)
{
    Arm_Observe(arm);
    Arm_GravityUpdate(arm);

    /* 使能帧周期性重发，防止电机掉使能 */
    if (arm->state == ARM_ENABLE || arm->state == ARM_HOST)
    {
        if (arm->enable_divider == 0U)
        {
            Arm_MotorsCall(arm, &MotorOps_t::enable);
            arm->enable_divider = ARM_ENABLE_RESEND_DIV;
        }
        --arm->enable_divider;
    }

    Arm_UpdateTarget(arm);
    Arm_JointControl(arm);
}

bool Arm_SetState(Arm_t *arm, ArmState_t state)
{
    if (!Arm_StateAllowed(arm, state))
    {
        return false;
    }
    if (state == arm->state)
    {
        return true;
    }

    if (state == ARM_PROTECTED)
    {
        Arm_MotorsCall(arm, &MotorOps_t::disable);
    }
    arm->state = state;
    arm->enable_divider = 0U;
    arm->ctrl_align_pending = 1U;
    arm->host_align_pending = state == ARM_HOST ? 1U : 0U;
    arm->host_closed_loop_mask = 0U;
    arm->host_target_valid_mask = 0U;
    return true;
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
    bit = (uint8_t)(1U << joint);
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
        Arm_AlignHost(arm);
    }

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
    memset(arm->gravity.torque_cmd, 0, sizeof(arm->gravity.torque_cmd));
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

        /* 当前原始角 = multi_angle_rad + zero_point_rad，把零点挪到当前姿态 */
        motor->zero_point_rad += motor->feedback.multi_angle_rad;
        arm->config.param.zero_point_rad[i] = motor->zero_point_rad;
    }
    return true;
}
