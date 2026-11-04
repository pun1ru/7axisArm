#include "motor.h"

#include <math.h>
#include <string.h>

#define SIM_ENCODER_CPR   65536.0f

static SimMotor_t *SimMotor_FromBase(Motor_t *motor)
{
    /* base 是 SimMotor_t 的第一个成员 */
    return (SimMotor_t *)motor;
}

static float SimMotor_WrapPi(float angle_rad)
{
    angle_rad = fmodf(angle_rad + MOTOR_PI, MOTOR_TWO_PI);
    if (angle_rad < 0.0f)
    {
        angle_rad += MOTOR_TWO_PI;
    }
    return angle_rad - MOTOR_PI;
}

/*--------------------------------------电机操作--------------------------------------*/
static void SimMotor_Enable(Motor_t *motor)
{
    SimMotor_t *sim = SimMotor_FromBase(motor);

    sim->enabled = 1U;
    motor->feedback.status = MOTOR_ENABLED;
}

static void SimMotor_Disable(Motor_t *motor)
{
    SimMotor_t *sim = SimMotor_FromBase(motor);

    sim->enabled = 0U;
    sim->command_torque = 0.0f;
    motor->feedback.status = MOTOR_DISABLED;
}

static void SimMotor_Protect(Motor_t *motor)
{
    SimMotor_Disable(motor);
}

/* 真机在这里按控制模式组 CAN 帧；仿真只把指令解析成关节力矩缓存下来，
 * 由 System 的 PlantTask 统一写进物理引擎，时序与“ArmTask 写指令 / 发送任务发帧”一致 */
static void SimMotor_SendCommand(Motor_t *motor)
{
    SimMotor_t *sim = SimMotor_FromBase(motor);
    const MotorCmd_t *cmd = &motor->motorcmd;
    float torque = 0.0f;

    if (sim->enabled == 0U)
    {
        sim->command_torque = 0.0f;
        return;
    }

    switch (cmd->control_mode)
    {
    case MODE_MIT:
    case MODE_TORQUE:
        torque = cmd->torque;
        break;
    case MODE_CURRENT:
        torque = cmd->current * (motor->torque_constant > 0.0f ? motor->torque_constant : 1.0f);
        break;
    case MODE_POSITION:
    case MODE_VELOCITY:
    default:
        torque = 0.0f;
        break;
    }
    sim->command_torque = torque * Motor_Direction(motor);
}

static void SimMotor_RequestFeedback(Motor_t *motor)
{
    (void)motor;
}

/*--------------------------------------构造与反馈--------------------------------------*/
void Motor_BaseInit(Motor_t *motor, float zero_point_rad, bool inverse, float gear_ratio)
{
    motor->zero_point_rad = zero_point_rad;
    motor->inverse = inverse;
    motor->gear_ratio = gear_ratio > 0.0f ? gear_ratio : 1.0f;
    motor->torque_constant = 0.0f;
    motor->multi_turn_count = 0;
    motor->last_abs_angle_rad = 0.0f;
    motor->last_feedback_ms = 0U;
    motor->feedback_angle_valid = 0U;
    motor->feedback.status = MOTOR_DISABLED;
}

void Motor_UpdateFeedbackStatus(Motor_t *motor)
{
    (void)motor;
}

void SimMotor_Init(SimMotor_t *motor, const SimMotorConfig_t *config)
{
    if (motor == NULL || config == NULL || config->plant == NULL)
    {
        return;
    }

    memset(motor, 0, sizeof(*motor));
    Motor_BaseInit(&motor->base, config->zero_point_rad, config->inverse, config->gear_ratio);
    motor->base.torque_constant = config->torque_constant;
    motor->base.motorcmd.control_mode = config->control_mode;
    motor->base.motorops.enable = SimMotor_Enable;
    motor->base.motorops.disable = SimMotor_Disable;
    motor->base.motorops.protect = SimMotor_Protect;
    motor->base.motorops.sendcommand = SimMotor_SendCommand;
    motor->base.motorops.requestfeedback = SimMotor_RequestFeedback;
    motor->plant = config->plant;
    motor->joint_index = config->joint_index;
    motor->enabled = 0U;
    motor->command_torque = 0.0f;

    SimMotor_UpdateFeedback(motor);
}

void SimMotor_UpdateFeedback(SimMotor_t *motor)
{
    Motor_t *base;
    Feedback_t *fb;
    float angle_rad;
    float abs_rad;

    if (motor == NULL || motor->plant == NULL)
    {
        return;
    }

    base = &motor->base;
    fb = &base->feedback;

    angle_rad = (float)SimPlant_GetJointAngleRad(motor->plant, motor->joint_index) -
                base->zero_point_rad;
    abs_rad = SimMotor_WrapPi(angle_rad);

    fb->timestamp = motor->feedback_ticks++;
    fb->encoder = (uint16_t)((int32_t)lroundf(abs_rad / MOTOR_TWO_PI * SIM_ENCODER_CPR) &
                             (int32_t)(SIM_ENCODER_CPR - 1.0f));
    fb->abs_angle_rad = abs_rad;
    fb->abs_angle_deg = abs_rad * MOTOR_RAD_TO_DEG;
    fb->multi_angle_rad = angle_rad;
    fb->multi_angle_deg = angle_rad * MOTOR_RAD_TO_DEG;
    fb->omega_rads = (float)SimPlant_GetJointVelocityRad(motor->plant, motor->joint_index);
    fb->velocity_rpm = fb->omega_rads * MOTOR_RADS_TO_RPM;
    fb->torque_nm = motor->command_torque;
    fb->current = (base->torque_constant > 0.0f) ? fb->torque_nm / base->torque_constant
                                                 : fb->torque_nm;
    fb->temp = 35.0f;
    fb->status = (motor->enabled != 0U) ? MOTOR_ENABLED : MOTOR_DISABLED;

    base->last_abs_angle_rad = abs_rad;
    base->last_feedback_ms = fb->timestamp;
    base->feedback_angle_valid = 1U;
}

void SimMotor_WriteToPlant(SimMotor_t *motor)
{
    if (motor == NULL || motor->plant == NULL)
    {
        return;
    }
    SimPlant_SetJointTorque(motor->plant, motor->joint_index, motor->command_torque);
}
