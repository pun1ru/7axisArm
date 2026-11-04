#include "system.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

#include "system_define.h"

/*--------------------------------------实例--------------------------------------*/
SimPlant_t simPlant;
Arm_t armInstance;

static SimMotor_t s_motors[ARM_JOINT_COUNT];

/*--------------------------------------任务表--------------------------------------*/
#define SYSTEM_TASK_COUNT   3U

typedef struct {
    const char *name;
    uint32_t period_ms;
    double next_s;
    void (*run)(void);
} SystemTask_t;

static void ArmTask(void);
static void PlantTask(void);
static void MonitorTask(void);

/* 顺序即同一 tick 内的执行顺序：先控制，再步进 */
static SystemTask_t s_tasks[SYSTEM_TASK_COUNT] = {
    {"arm",     TASK_ARM_PERIOD_MS,     0.0, ArmTask},
    {"plant",   TASK_PLANT_PERIOD_MS,   0.0, PlantTask},
    {"monitor", TASK_MONITOR_PERIOD_MS, 0.0, MonitorTask},
};

/*--------------------------------------初始化--------------------------------------*/
static bool System_BaseInit(const char *xml_path)
{
    SimPlantConfig_t config;

    config.xml_path = xml_path;
    config.joint_armature = SIM_JOINT_ARMATURE;
    config.joint_damping = SIM_JOINT_DAMPING;
    config.joint_frictionloss = SIM_JOINT_FRICTIONLOSS;
    config.torque_limit_nm = SIM_TORQUE_LIMIT_NM;

    if (!SimPlant_Init(&simPlant, &config))
    {
        return false;
    }
    SimPlant_Reset(&simPlant);
    return true;
}

/* 仿真电机与模型关节同轴：零点 0、方向 +1、inverse false。
 * 真机的装配标定值（编码器零点、反装电机）在这里不需要。 */
static void System_MotorInit(void)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        SimMotorConfig_t config;

        memset(&config, 0, sizeof(config));
        config.plant = &simPlant;
        config.joint_index = i;
        config.zero_point_rad = 0.0f;
        config.inverse = false;
        config.gear_ratio = 1.0f;
        config.torque_constant = 1.0f;
        config.control_mode = MODE_TORQUE;
        SimMotor_Init(&s_motors[i], &config);
    }
}

/* 改进 DH，一行对应 J0..J6，末端坐标系为 {J6}，d 取自 DIY/dh_params.md */
static const DH_Param_t kDhTable[ARM_JOINT_COUNT] = {
    {0.0f,  0.0f,  0.0626f,             0.0f},
    {90.0f, 0.0f, -0.00400004092454f,   0.0f},
    {90.0f, 0.0f, -0.147999608f,        0.0f},
    {90.0f, 0.0f, -0.00249973171439f,   0.0f},
    {90.0f, 0.0f,  0.0999988885966f,    0.0f},
    {90.0f, 0.0f, -0.000999402709942f,  0.0f},
    {90.0f, 0.0f,  0.0f,                0.0f},
};

static const ArmJointLimit_t kLimits[ARM_JOINT_COUNT] = {
    {-90.0f, 90.0f},
    {-95.0f, 100.0f},
    {-180.0f, 180.0f},
    {-120.0f, 0.0f},
    {-180.0f, 180.0f},
    {-60.0f, 90.0f},
    {-180.0f, 180.0f},
};

/* kp, ki, kd, d_alpha, sum_error_max, output_max, total_output_max
 * 误差 deg -> 力矩 N*m，与实机 module_init.c 相同 */
static const ArmJointGain_t kGains[ARM_JOINT_COUNT] = {
    {0.1f,   0.0f, 0.0f,   0.0f, 0.0f, 0.2f,  0.2f},
    {0.5f,   0.0f, 0.0f,   0.0f, 0.0f, 2.5f,  2.5f},
    {0.1f,   0.0f, 0.0f,   0.0f, 0.0f, 0.5f,  0.5f},
    {0.1f,   0.0f, 0.0f,   0.0f, 0.0f, 0.6f,  0.6f},
    {0.002f, 0.0f, 0.0f,   0.0f, 0.0f, 0.25f, 0.25f},
    {0.003f, 0.0f, 0.0f,   0.0f, 0.0f, 0.2f,  0.2f},
    {0.002f, 0.0f, 0.001f, 0.0f, 0.0f, 0.05f, 0.05f},
};

/* 12 维重力参数 beta = [h2x h2y ... h7x h7y]，来自实机重力辨识 */
static const float kGravityBeta[ARM_GRAVITY_BETA_DIM] = {
    -0.00052f, +0.1739178400f,
    -0.00375f, +0.00024f,
    +0.00040f, -0.029266f,
    +0.00053f, +0.00024f,
    +0.00137f, +0.00916f,
    -0.00116f, +0.00064f,
};

static void System_ModuleInit(void)
{
    ArmConfig_t config;

    memset(&config, 0, sizeof(config));

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        config.motor.joint[i] = &s_motors[i].base;
        config.param.direction[i] = 1;
        config.param.zero_point_rad[i] = 0.0f;
    }
    memcpy(config.param.dh, kDhTable, sizeof(kDhTable));
    memcpy(config.param.limit, kLimits, sizeof(kLimits));
    memcpy(config.param.gain, kGains, sizeof(kGains));

    config.state.role = ARM_ROLE_MASTER;
    config.state.init_state = ARM_PROTECTED;

    /* J0 绕竖直轴，重力矩恒为 0，实机也没开 */
    config.param.gravity.enable = 1U;
    config.param.gravity.use_beta = 1U;
    config.param.gravity.sign = 1;
    config.param.gravity.joint_mask = (uint8_t)(ARM_GRAVITY_ALL_JOINTS & ~0x01U);
    config.param.gravity.g = ARM_GRAVITY_DEFAULT_G;
    config.param.gravity.pi = NULL;
    config.param.gravity.beta = kGravityBeta;

    Arm_Init(&armInstance, &config);
}

static void System_TaskCreate(void)
{
    for (uint32_t i = 0U; i < SYSTEM_TASK_COUNT; ++i)
    {
        s_tasks[i].next_s = 0.0;
    }
}

/*--------------------------------------任务--------------------------------------*/
/* 观测：物理引擎状态 -> 电机反馈对象，相当于 CAN 中断已经收完一帧 */
static void ArmTask(void)
{
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        SimMotor_UpdateFeedback(&s_motors[i]);
    }
    Arm_Run(&armInstance);
}

/* 通信 + 物理步进：motorcmd -> qfrc_applied -> mj_step */
static void PlantTask(void)
{
    SimPlant_ClearTorque(&simPlant);
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        SimMotor_WriteToPlant(&s_motors[i]);
    }
    SimPlant_Step(&simPlant);
}

static const char *Arm_StateName(ArmState_t state)
{
    switch (state)
    {
    case ARM_PROTECTED: return "PROTECTED";
    case ARM_ENABLE:    return "ENABLE";
    case ARM_FEEDBACK:  return "FEEDBACK";
    case ARM_MASTER:    return "MASTER";
    case ARM_AUTO:      return "AUTO";
    case ARM_SERVANT:   return "SERVANT";
    case ARM_HOST:      return "HOST";
    default:            return "?";
    }
}

static void MonitorTask(void)
{
    const ArmObserver_t *obs = &armInstance.observer;
    double tcp[3] = {0.0, 0.0, 0.0};
    float tau_max = 0.0f;

    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
    {
        const float tau = armInstance.joint_ctrl[i].output;
        if (fabsf(tau) > tau_max)
        {
            tau_max = fabsf(tau);
        }
    }
    (void)SimPlant_GetTcpPosition(&simPlant, tcp);

    printf("t=%5.2f  %-9s  q=[%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f]  "
           "tcp=[%6.3f %6.3f %6.3f]  |tau|max=%6.3f  warn=%d\n",
           SimPlant_GetTime(&simPlant), Arm_StateName(armInstance.state),
           obs->joint_deg[0], obs->joint_deg[1], obs->joint_deg[2], obs->joint_deg[3],
           obs->joint_deg[4], obs->joint_deg[5], obs->joint_deg[6],
           tcp[0], tcp[1], tcp[2], tau_max, SimPlant_GetWarningCount(&simPlant));
}

/*--------------------------------------仿真输入--------------------------------------*/
/* 替代实机的遥控器和上位机。接自己的算法时改这里，Arm_Run 不用动。 */
static uint8_t s_enable_sent;
static uint8_t s_host_sent;
static uint8_t s_return_sent;

static void System_InputRun(double time_s)
{
    /* ENABLE 只是过渡：这个状态的目标每周期跟随观测，靠重力前馈托住；
     * 仿真没有真实关节摩擦，停留太久会慢慢漂。0.5 s 就切到全关节闭环的 HOST。 */
    const double enable_s = 0.2;
    const double host_s = 0.5;
    const double return_s = 3.5;

    if (!s_enable_sent && time_s >= enable_s)
    {
        s_enable_sent = 1U;
        if (Arm_SetState(&armInstance, ARM_ENABLE))
        {
            printf("[input] t=%.2f -> ENABLE\n", time_s);
        }
    }

    if (!s_host_sent && time_s >= host_s)
    {
        s_host_sent = 1U;
        if (Arm_SetState(&armInstance, ARM_HOST))
        {
            /* 先让所有关节闭环在当前角度，再给 J1 一个目标 */
            for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
            {
                (void)Arm_SetHostTarget(&armInstance, i,
                                        armInstance.observer.joint_abs_deg[i], true);
            }
            (void)Arm_SetHostTarget(&armInstance, ARM_J1, 20.0f, true);
            printf("[input] t=%.2f -> HOST (所有关节闭环), J1 target +20 deg\n", time_s);
        }
    }

    if (!s_return_sent && time_s >= return_s)
    {
        s_return_sent = 1U;
        if (armInstance.state == ARM_HOST)
        {
            (void)Arm_SetHostTarget(&armInstance, ARM_J1, 0.0f, true);
            printf("[input] t=%.2f -> J1 target 0 deg\n", time_s);
        }
    }
}

/*--------------------------------------对外接口--------------------------------------*/
bool System_Init(const char *xml_path)
{
    if (!System_BaseInit(xml_path))
    {
        return false;
    }
    System_MotorInit();
    System_ModuleInit();
    System_TaskCreate();
    s_enable_sent = 0U;
    s_host_sent = 0U;
    s_return_sent = 0U;

    printf("[system] model=%s, %d joints, dt=%.4f s\n",
           xml_path, SimPlant_GetJointCount(&simPlant), SimPlant_GetTimestep(&simPlant));
    printf("[system] tasks: arm=%u ms, plant=%u ms, monitor=%u ms\n",
           (unsigned int)TASK_ARM_PERIOD_MS, (unsigned int)TASK_PLANT_PERIOD_MS,
           (unsigned int)TASK_MONITOR_PERIOD_MS);
    printf("[system] 0.2 s ENABLE, 0.5 s HOST + J1 20 deg, 3.5 s back to 0\n");
    return true;
}

void System_Run(double duration_s, bool realtime)
{
    const double dt = SimPlant_GetTimestep(&simPlant);
    const double expected_dt = (double)TASK_PLANT_PERIOD_MS * 0.001;
    const auto wall_start = std::chrono::steady_clock::now();

    if (dt <= 0.0)
    {
        printf("[system] 无效 timestep\n");
        return;
    }
    if (fabs(dt - expected_dt) > 1e-9)
    {
        printf("[system] 注意: MJCF timestep=%.6f s, PlantTask 周期=%.6f s\n", dt, expected_dt);
    }

    while (SimPlant_GetTime(&simPlant) < duration_s)
    {
        const double now = SimPlant_GetTime(&simPlant);

        System_InputRun(now);

        /* 到期任务按表顺序跑一次 */
        for (uint32_t i = 0U; i < SYSTEM_TASK_COUNT; ++i)
        {
            SystemTask_t *task = &s_tasks[i];

            if (now + 1e-9 >= task->next_s)
            {
                task->run();
                task->next_s += (double)task->period_ms * 0.001;
                if (task->next_s <= now)
                {
                    task->next_s = now + (double)task->period_ms * 0.001;
                }
            }
        }

        if (realtime)
        {
            const auto target = wall_start +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(SimPlant_GetTime(&simPlant)));
            std::this_thread::sleep_until(target);
        }
    }

    printf("[system] done: t=%.2f s, steps=%u, warnings=%d\n",
           SimPlant_GetTime(&simPlant), (unsigned int)simPlant.step_count,
           SimPlant_GetWarningCount(&simPlant));
}

void System_Deinit(void)
{
    SimPlant_Clear(&simPlant);
}
