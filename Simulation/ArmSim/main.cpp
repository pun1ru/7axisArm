#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <thread>

#include "arm.h"
#include "host.h"
#include "host_link.h"
#include "mujoco_world.h"

#ifndef ARMSIM_DEFAULT_XML
#define ARMSIM_DEFAULT_XML "../mujoco/DIY/robot.xml"   // CMake 会覆盖成源码树里的绝对路径
#endif

namespace {

/*--------------------------------------仿真参数--------------------------------------*/
constexpr double kControlPeriodS = 0.002;       // 必须等于 MJCF 的 timestep
constexpr int kMonitorEverySteps = 50;          // 100 ms
constexpr double kDefaultDurationS = 8.0;
constexpr const char *kHostLinkPath = "/tmp/armsim-host";
constexpr uint16_t kHostStateDivider = 5;       // 2 ms * 5 = 100 Hz，与实机相同

/* 仿真专用的电机参数，不属于 URDF。DIY/robot.xml 里没有这三项，远端关节惯量
 * 只有 1.5e-5 kg*m^2，纯力矩控制会抖；加上后与实机手感接近。置 0 就是“纯 MJCF”。 */
constexpr double kJointArmature = 1.0e-3;       // kg*m^2
constexpr double kJointDamping = 1.5;           // N*m*s/rad
constexpr double kJointFrictionLoss = 0.15;     // N*m
constexpr double kTorqueLimitNm = 20.0;         // 单关节安全限幅

volatile std::sig_atomic_t g_stop = 0;

void OnSignal(int)
{
    g_stop = 1;
}

/*--------------------------------------机械臂参数--------------------------------------*/
/* 增益、限位、DH、beta 全部来自实机。仿真电机与模型关节同轴：零点 0、方向 +1，
 * 真机的装配标定值（编码器零点、反装电机）在这里不需要。 */
Arm::Config MakeArmConfig()
{
    Arm::Config c;

    c.role = Arm::Role::Master;
    c.init_state = Arm::State::Protected;

    /* 改进 DH，一行对应 J0..J6，末端坐标系为 {J6}，d 取自 DIY/dh_params.md */
    c.dh = {{
        {0.0f,  0.0f,  0.0626f,             0.0f},
        {90.0f, 0.0f, -0.00400004092454f,   0.0f},
        {90.0f, 0.0f, -0.147999608f,        0.0f},
        {90.0f, 0.0f, -0.00249973171439f,   0.0f},
        {90.0f, 0.0f,  0.0999988885966f,    0.0f},
        {90.0f, 0.0f, -0.000999402709942f,  0.0f},
        {90.0f, 0.0f,  0.0f,                0.0f},
    }};
    c.direction.fill(1);
    c.zero_point_rad.fill(0.0f);

    c.limit = {{
        {-90.0f, 90.0f},
        {-95.0f, 100.0f},
        {-180.0f, 180.0f},
        {-120.0f, 0.0f},
        {-180.0f, 180.0f},
        {-60.0f, 90.0f},
        {-180.0f, 180.0f},
    }};

    /* kp, ki, kd, d_alpha, sum_error_max, output_max, total_output_max
     * 误差 deg -> 力矩 N*m，与实机 module_init.c 相同 */
    const float gain[kJointCount][7] = {
        {0.1f,   0.0f, 0.0f,   0.0f, 0.0f, 0.2f,  0.2f},
        {0.5f,   0.0f, 0.0f,   0.0f, 0.0f, 2.5f,  2.5f},
        {0.1f,   0.0f, 0.0f,   0.0f, 0.0f, 0.5f,  0.5f},
        {0.1f,   0.0f, 0.0f,   0.0f, 0.0f, 0.6f,  0.6f},
        {0.002f, 0.0f, 0.0f,   0.0f, 0.0f, 0.25f, 0.25f},
        {0.003f, 0.0f, 0.0f,   0.0f, 0.0f, 0.2f,  0.2f},
        {0.002f, 0.0f, 0.001f, 0.0f, 0.0f, 0.05f, 0.05f},
    };
    for (int i = 0; i < kJointCount; ++i)
    {
        c.gain[i].kp = gain[i][0];
        c.gain[i].ki = gain[i][1];
        c.gain[i].kd = gain[i][2];
        c.gain[i].d_alpha = gain[i][3];
        c.gain[i].sum_error_max = gain[i][4];
        c.gain[i].output_max = gain[i][5];
        c.gain[i].total_output_max = gain[i][6];
    }

    /* 12 维重力参数 beta = [h2x h2y ... h7x h7y]，来自实机重力辨识。
     * J0 绕竖直轴，重力矩恒为 0，实机也没开 */
    c.gravity.enable = true;
    c.gravity.sign = 1;
    c.gravity.joint_mask = GravityModel::kAllJoints & ~0x01U;
    c.gravity.g = GravityModel::kDefaultG;
    c.gravity.beta = {
        -0.00052f, +0.1739178400f,
        -0.00375f, +0.00024f,
        +0.00040f, -0.029266f,
        +0.00053f, +0.00024f,
        +0.00137f, +0.00916f,
        -0.00116f, +0.00064f,
    };
    return c;
}

/*--------------------------------------仿真输入--------------------------------------*/
/* 替代实机的遥控器。按时间排序，每项只触发一次。接自己的算法时改这里，Arm 不用动。
 * 仿真没有真实关节摩擦，ENABLE 停留太久会慢慢漂，所以 0.5 s 就切到全关节闭环的 HOST。 */
struct InputEvent {
    double time_s;
    void (*run)(Arm &arm, double time_s);
};

void InputEnable(Arm &arm, double time_s)
{
    /* ENABLE 只是过渡：这个状态的目标每周期跟随观测，靠重力前馈托住 */
    if (arm.SetState(Arm::State::Enable))
    {
        std::printf("[input] t=%.2f -> ENABLE\n", time_s);
    }
}

void InputHost(Arm &arm, double time_s)
{
    if (arm.SetState(Arm::State::Host))
    {
        /* 先让所有关节闭环在当前角度，再给 J1 一个目标 */
        for (int i = 0; i < kJointCount; ++i)
        {
            arm.SetHostTarget(i, arm.observer().joint_abs_deg[i], true);
        }
        arm.SetHostTarget(1, 20.0f, true);
        std::printf("[input] t=%.2f -> HOST (所有关节闭环), J1 target +20 deg\n", time_s);
    }
}

void InputReturn(Arm &arm, double time_s)
{
    if (arm.state() == Arm::State::Host)
    {
        arm.SetHostTarget(1, 0.0f, true);
        std::printf("[input] t=%.2f -> J1 target 0 deg\n", time_s);
    }
}

constexpr InputEvent kInputEvents[] = {
    {0.2, InputEnable},
    {0.5, InputHost},
    {3.5, InputReturn},
};

/*--------------------------------------监视--------------------------------------*/
void PrintStatus(const MujocoWorld &world, const Arm &arm)
{
    const Arm::Observer &obs = arm.observer();
    const auto tcp = world.TcpPosition().value_or(std::array<double, 3>{});
    float tau_max = 0.0f;

    for (int i = 0; i < kJointCount; ++i)
    {
        tau_max = std::fmax(tau_max, std::fabs(arm.joint(i).output));
    }
    std::printf("t=%5.2f  %-9s  q=[%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f]  "
                "tcp=[%6.3f %6.3f %6.3f]  |tau|max=%6.3f  warn=%d\n",
                world.time(), ToString(arm.state()),
                obs.joint_deg[0], obs.joint_deg[1], obs.joint_deg[2], obs.joint_deg[3],
                obs.joint_deg[4], obs.joint_deg[5], obs.joint_deg[6],
                tcp[0], tcp[1], tcp[2], tau_max, world.warning_count());
}

void PrintUsage(const char *prog)
{
    std::printf("用法: %s [模型.xml] [秒数] [--fast] [--host]\n"
                "  默认模型 %s\n"
                "  --fast  不按实时节拍 sleep\n"
                "  --host  开伪串口 %s 等 Host/armhost 连接，由上位机给目标，Ctrl-C 退出\n",
                prog, ARMSIM_DEFAULT_XML, kHostLinkPath);
}

}  // namespace

int main(int argc, char **argv)
{
    const char *xml_path = ARMSIM_DEFAULT_XML;
    double duration_s = kDefaultDurationS;
    bool realtime = true;
    bool host_mode = false;
    bool duration_given = false;
    int positional = 0;

    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--fast") == 0)
        {
            realtime = false;
        }
        else if (std::strcmp(argv[i], "--host") == 0)
        {
            host_mode = true;
        }
        else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0 || positional >= 2)
        {
            PrintUsage(argv[0]);
            return positional >= 2 ? 1 : 0;
        }
        else if (positional++ == 0)
        {
            xml_path = argv[i];
        }
        else
        {
            duration_s = std::atof(argv[i]);
            duration_given = true;
        }
    }
    if (host_mode)
    {
        realtime = true;   // 上位机是真实时间，仿真不能跑快
        if (!duration_given)
        {
            duration_s = 1.0e9;   // 交互用，没指定时长就一直跑
        }
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    try
    {
        /* 构造：物理世界 -> 机械臂 ->（可选）上位机链路和协议桥 */
        MujocoWorld::Config world_config;
        world_config.xml_path = xml_path;
        world_config.joint_armature = kJointArmature;
        world_config.joint_damping = kJointDamping;
        world_config.joint_frictionloss = kJointFrictionLoss;
        world_config.torque_limit_nm = kTorqueLimitNm;

        MujocoWorld world(world_config);
        Arm arm(world, MakeArmConfig());

        std::unique_ptr<HostLink> link;
        std::unique_ptr<Host> host;
        if (host_mode)
        {
            link = std::make_unique<HostLink>(kHostLinkPath);
            host = std::make_unique<Host>(arm, *link, kHostStateDivider);
            /* 进入 HOST 时没有任何关节闭环，只有重力前馈托着，等上位机发 SET_HOST_TARGET */
            arm.SetState(Arm::State::Host);
            std::printf("[system] host link: %s -> %s  (921600 8N1，Host/armhost 直接连这个口)\n",
                        link->link_path().c_str(), link->device_path().c_str());
        }

        const double dt = world.timestep();
        if (dt <= 0.0)
        {
            std::fprintf(stderr, "无效 timestep\n");
            return 1;
        }
        if (std::fabs(dt - kControlPeriodS) > 1e-9)
        {
            std::printf("[system] 注意: MJCF timestep=%.6f s, 控制周期=%.6f s\n", dt, kControlPeriodS);
        }
        std::printf("[system] model=%s, %d joints, dt=%.4f s\n", xml_path, world.joint_count(), dt);
        if (!host_mode)
        {
            std::printf("[system] 0.2 s ENABLE, 0.5 s HOST + J1 20 deg, 3.5 s back to 0\n");
        }

        /* 步进仿真：输入 -> 臂控制 -> 上位机收发 -> 物理步进 */
        const auto wall_start = std::chrono::steady_clock::now();
        size_t next_event = 0;
        int step = 0;

        while (world.time() < duration_s && !g_stop)
        {
            const double now = world.time();

            if (!host_mode)   // 目标来自上位机时不跑脚本
            {
                while (next_event < sizeof(kInputEvents) / sizeof(kInputEvents[0]) &&
                       now >= kInputEvents[next_event].time_s)
                {
                    kInputEvents[next_event++].run(arm, now);
                }
            }

            arm.Step();

            if (host)
            {
                /* 链路超时按墙钟算：--fast 下仿真时间比真实时间快，不能用来判掉线 */
                const auto elapsed = std::chrono::steady_clock::now() - wall_start;
                host->Step(now, static_cast<uint32_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()));
            }

            world.Step();

            if (!host_mode && step % kMonitorEverySteps == 0)   // 状态由上位机看，终端不刷屏
            {
                PrintStatus(world, arm);
            }
            ++step;

            if (realtime)
            {
                std::this_thread::sleep_until(wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                               std::chrono::duration<double>(world.time())));
            }
        }

        std::printf("[system] done: t=%.2f s, steps=%u, warnings=%d\n",
                    world.time(), world.step_count(), world.warning_count());
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "初始化失败: %s\n", e.what());
        return 1;
    }
    return 0;
}
