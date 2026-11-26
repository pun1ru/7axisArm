#include <stdio.h>
#include <string.h>

#include "LKMotor.h"

#define TEST_MAX_FRAMES 256U

typedef struct {
    uint16_t id;
    uint8_t data[CAN_FRAME_DATA_MAX];
} TestFrame_t;

static TestFrame_t g_frames[TEST_MAX_FRAMES];
static unsigned g_frame_count;
static uint32_t g_tick_ms;
static LKMotor_t g_motor;

uint32_t HAL_GetTick(void)
{
    return ++g_tick_ms;
}

int CANTransmit_Enqueue(FDCAN_HandleTypeDef *hcan, uint16_t std_id,
                        const uint8_t data[CAN_FRAME_DATA_MAX])
{
    (void)hcan;
    if (g_frame_count < TEST_MAX_FRAMES)
    {
        g_frames[g_frame_count].id = std_id;
        memcpy(g_frames[g_frame_count].data, data, CAN_FRAME_DATA_MAX);
        ++g_frame_count;
    }
    return 1;
}

void Motor_BaseInit(Motor_t *motor, FDCAN_HandleTypeDef *can,
                    uint16_t zero_point, bool inverse, float gear_ratio)
{
    motor->can = can;
    motor->zero_point = zero_point;
    motor->inverse = inverse;
    motor->gear_ratio = gear_ratio;
    motor->feedback.status = MOTOR_NO_FEEDBACK;
}

void Motor_UpdateFeedbackAngle(Motor_t *motor, float angle_rad)
{
    motor->feedback.abs_angle_rad = angle_rad;
    motor->feedback.abs_angle_deg = angle_rad * MOTOR_RAD_TO_DEG;
    motor->feedback.multi_angle_rad = angle_rad;
    motor->feedback.multi_angle_deg = motor->feedback.abs_angle_deg;
}

static void SetRawAngle(int64_t raw)
{
    uint8_t data[CAN_FRAME_DATA_MAX] = {0x92};
    for (unsigned i = 0U; i < 7U; ++i)
    {
        data[i + 1U] = (uint8_t)((uint64_t)raw >> (8U * i));
    }
    LKMotor_Feedback(&g_motor, data, g_motor.motor_config.feedback_id);
}

int main(void)
{
    char command[24];

    while (scanf("%23s", command) == 1)
    {
        if (strcmp(command, "init") == 0)
        {
            int model, motor_id, feedback_id, zero_point, inverse, mode;
            LKMotorConfig_t config;

            if (scanf("%d %d %d %d %d %d", &model, &motor_id, &feedback_id,
                      &zero_point, &inverse, &mode) != 6)
            {
                return 2;
            }
            memset(&config, 0, sizeof(config));
            config.model = (LKMotorModel_t)model;
            config.motor_id = (uint8_t)motor_id;
            config.feedback_id = (uint16_t)feedback_id;
            config.zero_point = (uint16_t)zero_point;
            config.inverse = inverse != 0;
            config.control_mode = (ControlMode_t)mode;
            LKMotor_Init(&g_motor, NULL, &config);
            g_frame_count = 0U;
        }
        else if (strcmp(command, "pos") == 0)
        {
            float position_rad, max_velocity_rads;
            if (scanf("%f %f", &position_rad, &max_velocity_rads) != 2) return 2;
            g_motor.base.motorcmd.control_mode = MODE_POSITION;
            g_motor.base.motorcmd.position = position_rad;
            g_motor.base.motorcmd.velocity = max_velocity_rads;
            LKMotor_SendCommand(&g_motor);
        }
        else if (strcmp(command, "torque") == 0)
        {
            float torque;
            if (scanf("%f", &torque) != 1) return 2;
            g_motor.base.motorcmd.control_mode = MODE_TORQUE;
            g_motor.base.motorcmd.torque = torque;
            LKMotor_SendCommand(&g_motor);
        }
        else if (strcmp(command, "req") == 0)
        {
            LKMotor_RequestFeedback(&g_motor);
        }
        else if (strcmp(command, "raw") == 0)
        {
            long long raw;
            if (scanf("%lld", &raw) != 1) return 2;
            SetRawAngle((int64_t)raw);
        }
        else if (strcmp(command, "rawid") == 0)
        {
            int id;
            long long raw;
            uint8_t data[CAN_FRAME_DATA_MAX] = {0x92};
            if (scanf("%d %lld", &id, &raw) != 2) return 2;
            for (unsigned i = 0U; i < 7U; ++i)
            {
                data[i + 1U] = (uint8_t)((uint64_t)raw >> (8U * i));
            }
            LKMotor_Feedback(&g_motor, data, (uint16_t)id);
        }
        else if (strcmp(command, "state") == 0)
        {
            int cmd, temperature, iq, speed, encoder;
            uint8_t data[CAN_FRAME_DATA_MAX];
            if (scanf("%d %d %d %d %d", &cmd, &temperature, &iq, &speed, &encoder) != 5)
            {
                return 2;
            }
            data[0] = (uint8_t)cmd;
            data[1] = (uint8_t)temperature;
            data[2] = (uint8_t)iq;
            data[3] = (uint8_t)(iq >> 8);
            data[4] = (uint8_t)speed;
            data[5] = (uint8_t)(speed >> 8);
            data[6] = (uint8_t)encoder;
            data[7] = (uint8_t)(encoder >> 8);
            LKMotor_Feedback(&g_motor, data, g_motor.motor_config.feedback_id);
        }
        else if (strcmp(command, "dump") == 0)
        {
            printf("%.9g %.9g %.9g %.9g %u %u %u\n",
                   g_motor.base.feedback.multi_angle_deg,
                   g_motor.base.feedback.abs_angle_deg,
                   g_motor.base.feedback.current,
                   g_motor.base.feedback.omega_rads,
                   g_motor.base.feedback.encoder,
                   g_motor.base.feedback.timestamp,
                   g_motor.base.feedback.status);
        }
        else if (strcmp(command, "ids") == 0)
        {
            printf("%u %u %u\n", g_motor.motor_config.feedback_id,
                   g_motor.base.feedback_id, g_motor.last_feedback_id);
        }
        else if (strcmp(command, "frames") == 0)
        {
            for (unsigned i = 0U; i < g_frame_count; ++i)
            {
                printf("%03X", g_frames[i].id);
                for (unsigned j = 0U; j < CAN_FRAME_DATA_MAX; ++j)
                {
                    printf(" %02X", g_frames[i].data[j]);
                }
                printf("\n");
            }
            printf("END\n");
        }
        else if (strcmp(command, "clearframes") == 0)
        {
            g_frame_count = 0U;
        }
    }
    return 0;
}
