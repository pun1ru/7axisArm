#include <stdio.h>
#include <string.h>

#include "Motor.h"

static uint32_t g_tick;

uint32_t HAL_GetTick(void)
{
    return g_tick;
}

int CAN_BindRx(CanInstance_t *instance, uint16_t id,
               CanRxHandler_t handler, void *context)
{
    (void)instance;
    (void)id;
    (void)handler;
    (void)context;
    return pdPASS;
}

int main(void)
{
    char command[16];
    Motor_t motor;

    memset(&motor, 0, sizeof(motor));
    while (scanf("%15s", command) == 1)
    {
        if (strcmp(command, "config") == 0)
        {
            float multi;
            float absolute_angle;
            unsigned timestamp;
            if (scanf("%f %f %u", &multi, &absolute_angle, &timestamp) != 3) return 2;
            memset(&motor, 0, sizeof(motor));
            motor.feedback.timestamp = timestamp;
            motor.last_feedback_timestamp = timestamp - 1U;
            motor.feedback.multi_angle_rad = multi;
            motor.feedback.multi_angle_deg = multi * MOTOR_RAD_TO_DEG;
            motor.feedback.abs_angle_rad = absolute_angle;
            motor.feedback.abs_angle_deg = absolute_angle * MOTOR_RAD_TO_DEG;
        }
        else if (strcmp(command, "tick") == 0)
        {
            if (scanf("%u", &g_tick) != 1) return 2;
        }
        else if (strcmp(command, "update") == 0)
        {
            Motor_UpdateFeedbackStatus(&motor);
        }
        else if (strcmp(command, "dump") == 0)
        {
            printf("%.9g %.9g %u %u\n",
                   motor.feedback.multi_angle_rad,
                   motor.feedback.abs_angle_rad,
                   motor.feedback.status,
                   motor.feedback_angle_valid);
        }
    }
    return 0;
}
