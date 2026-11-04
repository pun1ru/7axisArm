#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "sim_plant.h"

/* Driver 层：电机驱动封装。
 * Motor_t / MotorCmd_t / Feedback_t / MotorOps_t 与真机工程
 * Real/DIYcontroller/LibSE/Drivers/Motor/Motor.h 保持同样的用法；
 * 仿真里把“CAN 收发”换成“读写 SimPlant”，上层模块看不到 MuJoCo。
 *
 * 调用关系和真机一致：
 *   Module 写 motorcmd -> motorops.sendcommand()   只把力矩缓存下来
 *   System 的 PlantTask -> SimMotor_WriteToPlant() 把缓存写进物理引擎并步进
 */

typedef enum {
    MODE_POSITION,
    MODE_VELOCITY,
    MODE_TORQUE,
    MODE_CURRENT,
    MODE_MIT,
} ControlMode_t;

typedef enum {
    MOTOR_DISABLED,
    MOTOR_ENABLED,
    MOTOR_FAULT,
    MOTOR_OVERVOLTAGE,
    MOTOR_UNDERVOLTAGE,
    MOTOR_OVERCURRENT,
    MOTOR_MOS_OVERTEMP,
    MOTOR_COIL_OVERTEMP,
    MOTOR_COMM_LOST,
    MOTOR_OVERLOAD,
    MOTOR_NO_FEEDBACK
} MotorStatus_t;

typedef struct {
    ControlMode_t control_mode;
    float position;
    float velocity;
    float torque;
    float current;
    float kp;
    float kd;
} MotorCmd_t;

#define MOTOR_PI              3.141592653589793f
#define MOTOR_TWO_PI          (2.0f * MOTOR_PI)
#define MOTOR_RAD_TO_DEG      (180.0f / MOTOR_PI)
#define MOTOR_DEG_TO_RAD      (MOTOR_PI / 180.0f)
#define MOTOR_RADS_TO_RPM     (60.0f / MOTOR_TWO_PI)
#define MOTOR_RPM_TO_RADS     (MOTOR_TWO_PI / 60.0f)

typedef struct {
    uint32_t timestamp;
    uint8_t error_id;
    MotorStatus_t status;
    uint16_t encoder;          ///< 单圈编码器值（仿真按 16 bit 模拟）
    float abs_angle_rad;       ///< 相对零点的单圈角 (-pi, pi]
    float abs_angle_deg;
    float multi_angle_rad;     ///< 多圈角
    float multi_angle_deg;
    float velocity_rpm;
    float omega_rads;
    float torque_nm;
    float temp;
    float current;
} Feedback_t;

typedef struct Motor_t Motor_t;

typedef struct {
    void (*enable)(Motor_t *motor);
    void (*disable)(Motor_t *motor);
    void (*protect)(Motor_t *motor);
    void (*sendcommand)(Motor_t *motor);
    void (*sendgroupcommand)(Motor_t *motor);
    void (*requestfeedback)(Motor_t *motor);
    void (*clearerror)(Motor_t *motor);
    void (*getfeedback)(Motor_t *motor, const uint8_t *data, uint16_t rec_id);
} MotorOps_t;

struct Motor_t {
    Feedback_t feedback;
    MotorCmd_t motorcmd;
    MotorOps_t motorops;
    float zero_point_rad;      ///< 机械零位偏置 [rad]；真机是编码器计数，仿真用 rad
    bool inverse;              ///< 反转向矩指令，不影响反馈
    float gear_ratio;
    float torque_constant;     ///< N*m/A，0 表示不提供
    int32_t multi_turn_count;
    float last_abs_angle_rad;
    uint32_t last_feedback_ms;
    uint8_t feedback_angle_valid;
};

/* inverse 对应的方向系数 ±1 */
static inline float Motor_Direction(const Motor_t *motor)
{
    return motor->inverse ? -1.0f : 1.0f;
}

void Motor_BaseInit(Motor_t *motor, float zero_point_rad, bool inverse, float gear_ratio);
/* 真机用于检测反馈超时；仿真每个控制周期都有反馈，函数保留但不动作 */
void Motor_UpdateFeedbackStatus(Motor_t *motor);

/*--------------------------------------仿真电机--------------------------------------*/
typedef struct {
    Motor_t base;
    SimPlant_t *plant;
    uint8_t joint_index;
    uint8_t enabled;
    float command_torque;      ///< sendcommand 解析出的关节力矩 [N*m]
    uint32_t feedback_ticks;
} SimMotor_t;

typedef struct {
    SimPlant_t *plant;
    uint8_t joint_index;       ///< MuJoCo joint1..jointN，也就是 ArmJoint_t
    float zero_point_rad;
    bool inverse;
    float gear_ratio;
    float torque_constant;
    ControlMode_t control_mode;
} SimMotorConfig_t;

void SimMotor_Init(SimMotor_t *motor, const SimMotorConfig_t *config);
/* 从物理引擎刷新反馈（相当于收到一帧 CAN 反馈） */
void SimMotor_UpdateFeedback(SimMotor_t *motor);
/* 把缓存的力矩写进物理引擎（相当于 CAN 发送任务真正发帧） */
void SimMotor_WriteToPlant(SimMotor_t *motor);

#endif
