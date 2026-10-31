/* 主机端 Arm_Run 行为测试：真实编译 arm.c / arm_jointctrl.c / arm_kinematics.c / arm_dynamics.c / dh.c / pid.c / algorism.c，
 * 电机用桩，sendcommand 被替换成“把 motorcmd.torque 记下来”，所以读到的就是最终要发给电机的力矩
 *
 * 命令（stdin 逐行）：
 *   cfg  gravity_enable use_beta sign mask  gain_kp pid_max total_max  配置并 Arm_Init，pi/beta/DH 在驱动里写死
 *   state N                                                       Arm_SetState
 *   q    q0..q6                                                   写反馈角（deg，同时作为 multi 与 abs）
 *   host joint target closed                                      Arm_SetHostTarget
 *   stop                                                          Arm_StopHostControl
 *   run                                                           Arm_Run 一次，输出 7 个力矩 + 7 个 torque_cmd + model_valid
 *   dump                                                          输出 target_deg
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "arm.h"
#include "arm_jointctrl.h"

static const float PI28[28] = {
    +0.00000f, +0.00000f, -0.00000f, +0.00000f,
    -0.00052f, +0.08218f, -0.00000f, +0.00000f,
    -0.00375f, +0.00012f, -0.08218f, +0.01216f,
    +0.00040f, -0.01262f, -0.00012f, +0.01216f,
    +0.00053f, +0.00012f, +0.01262f, +0.01342f,
    +0.00137f, +0.00458f, -0.00012f, +0.01342f,
    -0.00116f, +0.00064f, -0.00458f, +0.01342f};
static const float BETA12[12] = {
    -0.00052f, +0.1739178400f, -0.00375f, +0.00024f, +0.00040f, -0.029266f,
    +0.00053f, +0.00024f, +0.00137f, +0.00916f, -0.00116f, +0.00064f};

static Motor_t g_motor[ARM_JOINT_COUNT];
static CanInstance_t g_can;
static float g_sent[ARM_JOINT_COUNT];
static Arm_t g_arm;

static void FakeSend(Motor_t *m)
{
    for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i)
        if (m == &g_motor[i]) g_sent[i] = m->motorcmd.torque;
}

int main(void)
{
    char cmd[16];

    memset(g_motor, 0, sizeof(g_motor));
    for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i)
    {
        g_motor[i].motorcmd.control_mode = MODE_TORQUE;
        g_motor[i].motorops.sendcommand = FakeSend;
        g_motor[i].feedback.status = MOTOR_ENABLED;
    }

    while (scanf("%15s", cmd) == 1)
    {
        if (strcmp(cmd, "cfg") == 0)
        {
            int en, ub, sign, mask;
            float kp, pid_max, total_max;
            ArmConfig_t c;
            if (scanf("%d %d %d %d %f %f %f", &en, &ub, &sign, &mask, &kp, &pid_max, &total_max) != 7) return 2;
            memset(&c, 0, sizeof(c));
            for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i)
            {
                c.motor.joint[i] = &g_motor[i];
                c.periph.can[i] = &g_can;
                c.param.direction[i] = 1;
                c.param.limit[i].min_deg = -180.0f;
                c.param.limit[i].max_deg = 180.0f;
                c.param.gain[i].kp = kp;
                c.param.gain[i].output_max = pid_max;
                c.param.gain[i].total_output_max = total_max;
            }
            static const DH_Param_t DH[7] = {{0, 0, 0.0626f, 0}, {90, 0, 0, 0}, {90, 0, -0.148f, 0}, {90, 0, 0, 0},
                                             {90, 0, 0.1f, 0}, {90, 0, 0, 0}, {90, 0, 0, 0}};
            memcpy(c.param.dh, DH, sizeof(DH));
            c.state.role = ARM_ROLE_MASTER;
            c.state.init_state = ARM_PROTECTED;
            c.param.gravity.enable = (uint8_t)en;
            c.param.gravity.use_beta = (uint8_t)ub;
            c.param.gravity.sign = (int8_t)sign;
            c.param.gravity.joint_mask = (uint8_t)mask;
            c.param.gravity.g = 9.81f;
            c.param.gravity.pi = PI28;
            c.param.gravity.beta = BETA12;
            Arm_Init(&g_arm, &c);
        }
        else if (strcmp(cmd, "state") == 0)
        {
            int s;
            if (scanf("%d", &s) != 1) return 2;
            printf("ok %d\n", Arm_SetState(&g_arm, (ArmState_t)s) ? 1 : 0);
        }
        else if (strcmp(cmd, "q") == 0)
        {
            for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i)
            {
                float v;
                if (scanf("%f", &v) != 1) return 2;
                g_motor[i].feedback.abs_angle_deg = v;
                g_motor[i].feedback.multi_angle_deg = v;
            }
        }
        else if (strcmp(cmd, "host") == 0)
        {
            int j, closed;
            float t;
            if (scanf("%d %f %d", &j, &t, &closed) != 3) return 2;
            printf("host %d\n", (int)Arm_SetHostTarget(&g_arm, (uint8_t)j, t, closed != 0));
        }
        else if (strcmp(cmd, "stop") == 0)
        {
            Arm_StopHostControl(&g_arm);
        }
        else if (strcmp(cmd, "run") == 0)
        {
            memset(g_sent, 0, sizeof(g_sent));
            Arm_Run(&g_arm);
            for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) printf("%.9g ", g_sent[i]);
            printf("| ");
            for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) printf("%.9g ", g_arm.gravity.torque_cmd[i]);
            printf("| %d %d\n", g_arm.gravity.model_valid, g_arm.gravity.output_active);
        }
        else if (strcmp(cmd, "dump") == 0)
        {
            for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) printf("%.9g ", g_arm.joint_ctrl[i].target_deg);
            printf("\n");
        }
    }
    return 0;
}
