#include "system.h"

#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstddef>
#include <cstring>
#include <thread>

#include "system_define.h"

/*--------------------------------------实例--------------------------------------*/
SimPlant_t simPlant;
Arm_t armInstance;
Host_t hostInstance;

static SimMotor_t s_motors[ARM_JOINT_COUNT];

/*--------------------------------------任务表--------------------------------------*/
#define SYSTEM_TASK_COUNT   4U

typedef struct {
    uint32_t period_ms;
    double next_s;
    void (*run)(void);
} SystemTask_t;

static void ArmTask(void);
static void HostTask(void);
static void PlantTask(void);
static void MonitorTask(void);

/* 顺序即同一 tick 内的执行顺序：先控制，再收发上位机，最后步进 */
static SystemTask_t s_tasks[SYSTEM_TASK_COUNT] = {
    {TASK_ARM_PERIOD_MS,     0.0, ArmTask},
    {TASK_HOST_PERIOD_MS,    0.0, HostTask},
    {TASK_PLANT_PERIOD_MS,   0.0, PlantTask},
    {TASK_MONITOR_PERIOD_MS, 0.0, MonitorTask},
};

static bool s_host_mode;
static HostLink_t s_host_link;
static volatile std::sig_atomic_t s_stop;
static const auto s_wall_start = std::chrono::steady_clock::now();

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

/* 上位机收发。链路超时按墙钟算：--fast 下仿真时间比真实时间快，不能用来判掉线 */
static void HostTask(void)
{
    if (!s_host_mode)
    {
        return;
    }
    const auto elapsed = std::chrono::steady_clock::now() - s_wall_start;
    const uint32_t wall_ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

    Host_Run(&hostInstance, SimPlant_GetTime(&simPlant), wall_ms);
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
    static const char *const kNames[] = {"PROTECTED", "ENABLE", "FEEDBACK", "MASTER",
                                         "AUTO", "SERVANT", "HOST"};

    return (unsigned)state < sizeof(kNames) / sizeof(kNames[0]) ? kNames[state] : "?";
}

static void MonitorTask(void)
{
    const ArmObserver_t *obs = &armInstance.observer;
    double tcp[3] = {0.0, 0.0, 0.0};
    float tau_max = 0.0f;

    if (s_host_mode)   /* 状态由上位机看，终端不刷屏 */
    {
        return;
    }

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
static void InputEnable(double time_s)
{
    /* ENABLE 只是过渡：这个状态的目标每周期跟随观测，靠重力前馈托住 */
    if (Arm_SetState(&armInstance, ARM_ENABLE))
    {
        printf("[input] t=%.2f -> ENABLE\n", time_s);
    }
}

static void InputHost(double time_s)
{
    if (Arm_SetState(&armInstance, ARM_HOST))
    {
        /* 先让所有关节闭环在当前角度，再给 J1 一个目标 */
        for (uint8_t i = 0U; i < ARM_JOINT_COUNT; ++i)
        {
            (void)Arm_SetHostTarget(&armInstance, i, armInstance.observer.joint_abs_deg[i], true);
        }
        (void)Arm_SetHostTarget(&armInstance, ARM_J1, 20.0f, true);
        printf("[input] t=%.2f -> HOST (所有关节闭环), J1 target +20 deg\n", time_s);
    }
}

static void InputReturn(double time_s)
{
    if (armInstance.state == ARM_HOST)
    {
        (void)Arm_SetHostTarget(&armInstance, ARM_J1, 0.0f, true);
        printf("[input] t=%.2f -> J1 target 0 deg\n", time_s);
    }
}

/* 按时间排序，每项只触发一次。仿真没有真实关节摩擦，ENABLE 停留太久会慢慢漂，
 * 所以 0.5 s 就切到全关节闭环的 HOST。 */
static const struct {
    double time_s;
    void (*run)(double time_s);
} kInputEvents[] = {
    {0.2, InputEnable},
    {0.5, InputHost},
    {3.5, InputReturn},
};
static size_t s_next_event;

static void System_InputRun(double time_s)
{
    if (s_host_mode)   /* 目标来自上位机，不跑脚本 */
    {
        return;
    }
    while (s_next_event < sizeof(kInputEvents) / sizeof(kInputEvents[0]) &&
           time_s >= kInputEvents[s_next_event].time_s)
    {
        kInputEvents[s_next_event++].run(time_s);
    }
}

/*--------------------------------------对外接口--------------------------------------*/
/* 上位机链路：开 pty，初始化协议桥，臂进入 HOST。
 * 进入 HOST 时没有任何关节闭环，只有重力前馈托着，等上位机发 SET_HOST_TARGET */
static bool System_HostInit(void)
{
    HostConfig_t config;

    if (!HostLink_Open(&s_host_link, HOST_LINK_PATH))
    {
        return false;
    }
    config.link = &s_host_link;
    config.state_divider = HOST_STATE_DIVIDER;
    Host_Init(&hostInstance, &config, &armInstance);
    (void)Arm_SetState(&armInstance, ARM_HOST);

    printf("[system] host link: %s -> %s  (921600 8N1，Host/armhost 直接连这个口)\n",
           s_host_link.link_path, s_host_link.path);
    return true;
}

bool System_Init(const char *xml_path, bool host_mode)
{
    if (!System_BaseInit(xml_path))
    {
        return false;
    }
    System_MotorInit();
    System_ModuleInit();
    s_next_event = 0U;
    s_stop = 0;
    s_host_mode = host_mode;

    printf("[system] model=%s, %d joints, dt=%.4f s\n",
           xml_path, SimPlant_GetJointCount(&simPlant), SimPlant_GetTimestep(&simPlant));
    printf("[system] tasks: arm=%u ms, host=%u ms, plant=%u ms, monitor=%u ms\n",
           (unsigned int)TASK_ARM_PERIOD_MS, (unsigned int)TASK_HOST_PERIOD_MS,
           (unsigned int)TASK_PLANT_PERIOD_MS, (unsigned int)TASK_MONITOR_PERIOD_MS);

    if (host_mode)
    {
        if (!System_HostInit())
        {
            fprintf(stderr, "上位机链路打开失败（仅支持 Linux / macOS）\n");
            return false;
        }
    }
    else
    {
        printf("[system] 0.2 s ENABLE, 0.5 s HOST + J1 20 deg, 3.5 s back to 0\n");
    }
    return true;
}

void System_Stop(void)
{
    s_stop = 1;
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

    if (s_host_mode)
    {
        realtime = true;   /* 上位机是真实时间，仿真不能跑快 */
    }

    while (SimPlant_GetTime(&simPlant) < duration_s && !s_stop)
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
    HostLink_Close(&s_host_link);
    SimPlant_Clear(&simPlant);
}
