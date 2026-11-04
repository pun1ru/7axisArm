#include "motor.h"

#include <math.h>
#include <string.h>

#define SIM_ENCODER_CPR   65536.0f

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
    motor->feedback.status = MOTOR_ENABLED;
}

/* 失能同时清掉缓存的力矩；protect 在仿真里与 disable 相同 */
static void SimMotor_Disable(Motor_t *motor)
{
    ((SimMotor_t *)motor)->command_torque = 0.0f;
    motor->feedback.status = MOTOR_DISABLED;
}

/* 实机在这里按控制模式组 CAN 帧；仿真只把指令解析成关节力矩缓存下来，
 * 由 System 的 PlantTask 统一写进物理引擎，时序与“ArmTask 写指令 / 发送任务发帧”一致 */
static void SimMotor_SendCommand(Motor_t *motor)
{
    const MotorCmd_t *cmd = &motor->motorcmd;
    float torque = 0.0f;

    if (motor->feedback.status == MOTOR_ENABLED)
    {
        if (cmd->control_mode == MODE_CURRENT)
        {
            torque = cmd->current * (motor->torque_constant > 0.0f ? motor->torque_constant : 1.0f);
        }
        else if (cmd->control_mode == MODE_TORQUE || cmd->control_mode == MODE_MIT)
        {
            torque = cmd->torque;
        }
    }
    ((SimMotor_t *)motor)->command_torque = torque * Motor_Direction(motor);
}

/*--------------------------------------构造与反馈--------------------------------------*/
void SimMotor_Init(SimMotor_t *motor, const SimMotorConfig_t *config)
{
    if (motor == NULL || config == NULL || config->plant == NULL)
    {
        return;
    }

    memset(motor, 0, sizeof(*motor));
    motor->base.zero_point_rad = config->zero_point_rad;
    motor->base.inverse = config->inverse;
    motor->base.torque_constant = config->torque_constant;
    motor->base.motorcmd.control_mode = config->control_mode;
    motor->base.motorops = {SimMotor_Enable, SimMotor_Disable, SimMotor_Disable, SimMotor_SendCommand};
    motor->plant = config->plant;
    motor->joint_index = config->joint_index;

    SimMotor_UpdateFeedback(motor);
}

void SimMotor_UpdateFeedback(SimMotor_t *motor)
{
    Feedback_t *fb;
    float angle_rad;
    float abs_rad;

    if (motor == NULL || motor->plant == NULL)
    {
        return;
    }

    fb = &motor->base.feedback;
    angle_rad = (float)SimPlant_GetJointAngleRad(motor->plant, motor->joint_index) -
                motor->base.zero_point_rad;
    abs_rad = SimMotor_WrapPi(angle_rad);

    fb->timestamp++;
    fb->encoder = (uint16_t)((int32_t)lroundf(abs_rad / MOTOR_TWO_PI * SIM_ENCODER_CPR) &
                             (int32_t)(SIM_ENCODER_CPR - 1.0f));
    fb->abs_angle_rad = abs_rad;
    fb->abs_angle_deg = abs_rad * MOTOR_RAD_TO_DEG;
    fb->multi_angle_rad = angle_rad;
    fb->multi_angle_deg = angle_rad * MOTOR_RAD_TO_DEG;
    fb->omega_rads = (float)SimPlant_GetJointVelocityRad(motor->plant, motor->joint_index);
    fb->velocity_rpm = fb->omega_rads * MOTOR_RADS_TO_RPM;
    fb->torque_nm = motor->command_torque;
    fb->current = motor->base.torque_constant > 0.0f ? fb->torque_nm / motor->base.torque_constant
                                                     : fb->torque_nm;
}

void SimMotor_WriteToPlant(SimMotor_t *motor)
{
    if (motor == NULL || motor->plant == NULL)
    {
        return;
    }
    SimPlant_SetJointTorque(motor->plant, motor->joint_index, motor->command_torque);
}
