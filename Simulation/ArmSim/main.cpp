#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "arm.h"
#include "exchange_station.h"
#include "host.h"
#include "host_link.h"
#include "mujoco_world.h"
#ifdef ARMSIM_HAS_VIEWER
#include "viewer.h"
#endif

#ifndef ARMSIM_DEFAULT_XML
#define ARMSIM_DEFAULT_XML "../mujoco/BigArm/robot.xml"   // CMake 会覆盖成源码树里的绝对路径
#endif

#ifndef ARMSIM_DEFAULT_ARENA
#define ARMSIM_DEFAULT_ARENA ""
#endif

#ifndef ARMSIM_DEFAULT_VIEW_ARENA
#define ARMSIM_DEFAULT_VIEW_ARENA ""
#endif

#ifndef ARMSIM_DEFAULT_EXCHANGE
#define ARMSIM_DEFAULT_EXCHANGE ""
#endif

namespace {

/*--------------------------------------仿真参数--------------------------------------*/
constexpr double kControlPeriodS = 0.002;       // 必须等于 MJCF 的 timestep
constexpr int kMonitorEverySteps = 50;          // 100 ms
constexpr const char *kHostLinkPath = "/tmp/armsim-host";
constexpr const char *kArenaModelPrefix = "arm/";
constexpr const char *kExchangeModelPrefix = "exchange/";
constexpr std::array<double, 3> kDefaultArmPosition{{5.0, -5.0, 0.01}};
/* O 为兑换站底座正面与战场地面交线中点（规则 5.4.3）。普通地面场景里地面就是 z=0；
 * 换成带高地的场地时用 --exchange-position 指定。 */
constexpr std::array<double, 3> kDefaultExchangePosition{{4.05, -5.0, 0.0}};
constexpr uint16_t kHostStateDivider = 5;       // 2 ms * 5 = 100 Hz，与实机相同
constexpr int kSyncEverySteps = 17;             // 每 17 步（34 ms）把状态同步给窗口一次

/* 仿真专用的电机参数，不属于 URDF。MJCF 里没有这三项；加上后远端小惯量关节不抖、能停住。
 * 置 0 就是“纯 MJCF”。 */
constexpr double kJointArmature = 1.0e-3;       // 转子反射惯量
constexpr double kJointDamping = 0.05;          // 粘性阻尼，主要阻尼由控制器的 kd 提供
constexpr double kJointFrictionLoss = 0.02;     // 库仑摩擦

volatile std::sig_atomic_t g_stop = 0;

void OnSignal(int)
{
    g_stop = 1;
}

/*--------------------------------------机械臂参数--------------------------------------*/
/* 误差 deg -> 力矩 N*m，按关节惯量取 wn≈15 rad/s、阻尼比≈0.9 算出（惯量取自 q=0 时的质量矩阵对角线）。
 * kd 是每周期误差增量的系数（周期并入 kd）。 */
Arm::JointGain MakeGain(float kp, float kd, float output_max)
{
    Arm::JointGain gain;

    gain.kp = kp;
    gain.kd = kd;
    gain.output_max = output_max;
    gain.total_output_max = output_max;
    return gain;
}

std::string ModelName(const MujocoWorld &world, const std::string &name)
{
    return world.model_prefix() + name;
}

/* BigArm：joint1 是腰部，只起直立作用；joint2..joint8 是主臂 7 个关节；两个滑动关节是夹爪。
 * 仿真电机与模型关节同轴：零点 0、方向 +1。 */
Arm::Config MakeArmConfig(const MujocoWorld &world)
{
    Arm::Config c;

    c.role = Arm::Role::Master;
    c.init_state = Arm::State::Protected;

    /* base -> 腰部连杆（link1）的安装位姿，来自 URDF joint1 的 origin */
    c.waist_joint = ModelName(world, "joint1");
    c.waist_mount = TransformFromXyzRpy({0.000558840342f, -0.0208529944f, 0.038f},
                                        {1.570796327f, -1.570796327f, 0.0f});
    c.waist_hold_deg = 0.0f;
    c.waist_gain = MakeGain(3.89f, 233.6f, 30.0f);

    c.joint_name = {
        ModelName(world, "joint2"),
        ModelName(world, "joint3"),
        ModelName(world, "joint4"),
        ModelName(world, "joint5"),
        ModelName(world, "joint6"),
        ModelName(world, "joint7"),
        ModelName(world, "joint8"),
    };
    c.joint_effort_limit = 100.0;   // URDF effort

    /* 改进 DH，一行对应 joint2..joint8，末端坐标系为 {link8}，取自 mujoco/BigArm/dh_params.md */
    c.dh = {{
        {0.0f,  0.4105f, -0.0897f,  -72.5122038f},
        {90.0f, 0.0f,     0.0f,     -90.0f},
        {90.0f, 0.0f,     0.188f,   -90.0f},
        {90.0f, 0.0f,    -0.0025f,   170.7537190f},
        {90.0f, 0.0f,     0.207f,    10.6949890f},
        {90.0f, 0.0f,     0.0f,     -148.8585943f},
        {90.0f, 0.0f,     0.0f,       0.0f},
    }};
    c.direction.fill(1);
    c.zero_point_rad.fill(0.0f);
    c.limit = {{
        {-90.0f,   90.0f},   // J0
        {-20.0f,  175.0f},   // J1
        {-175.0f, 175.0f},   // J2
        {-110.0f, 110.0f},   // J3
        {-175.0f, 175.0f},   // J4
        {-120.0f,  70.0f},   // J5
        {-175.0f, 175.0f},   // J6
    }};

    c.gain = {
        MakeGain(1.12f,   67.2f, 30.0f),
        MakeGain(1.10f,   65.9f, 20.0f),
        MakeGain(0.0299f,  1.79f, 5.0f),
        MakeGain(0.295f,  17.7f,  8.0f),
        MakeGain(0.0106f,  0.636f, 3.0f),
        MakeGain(0.0196f,  1.18f,  2.0f),
        MakeGain(0.00236f, 0.141f, 2.0f),
    };

    /* 夹爪：两指沿各自导轨移动，0 = 装配位姿（张开），0.015 m = 夹紧。误差 mm -> 每指力 N */
    c.gripper.finger_joint = {
        ModelName(world, "gripper_finger1_joint"),
        ModelName(world, "gripper_finger2_joint"),
    };
    c.gripper.open_m = 0.0f;
    c.gripper.closed_m = 0.015f;
    c.gripper.finger_effort_limit = 50.0;   // URDF effort
    c.gripper.gain.kp = 0.596f;
    c.gripper.gain.kd = 5.36f;
    c.gripper.gain.output_max = 50.0f;

    /* 重力前馈的连杆质量和质心直接取自模型，和物理引擎用同一份 CAD 数据。
     * link[0] 是随腰部转动的 link1，link[1..7] 是 link2..link8；夹爪两指并入 link8 */
    c.gravity.enable = true;
    c.gravity.sign = 1;
    c.gravity.joint_mask = GravityModel::kAllJoints;
    c.gravity.g = GravityModel::kDefaultG;
    for (int i = 0; i < GravityModel::kDof; ++i)
    {
        const MujocoWorld::Inertial in =
            world.BodyInertial(ModelName(world, "link" + std::to_string(i + 1)), false);

        c.gravity.link[i].mass_kg = static_cast<float>(in.mass_kg);
        for (int k = 0; k < 3; ++k)
        {
            c.gravity.link[i].com_m[k] = static_cast<float>(in.com_m[k]);
        }
    }
    GravityModel::LinkInertia &tip = c.gravity.link[GravityModel::kDof - 1];
    double tip_mass = tip.mass_kg;
    std::array<double, 3> tip_moment = {tip.mass_kg * tip.com_m[0], tip.mass_kg * tip.com_m[1],
                                        tip.mass_kg * tip.com_m[2]};
    for (const char *finger : {"link9", "link10"})
    {
        const MujocoWorld::Inertial in =
            world.BodyInertial(ModelName(world, finger), true);   // 折算到 link8 坐标系

        tip_mass += in.mass_kg;
        for (int k = 0; k < 3; ++k)
        {
            tip_moment[k] += in.mass_kg * in.com_m[k];
        }
    }
    tip.mass_kg = static_cast<float>(tip_mass);
    for (int k = 0; k < 3; ++k)
    {
        tip.com_m[k] = static_cast<float>(tip_moment[k] / tip_mass);
    }
    return c;
}

/*--------------------------------------仿真输入--------------------------------------*/
/* 替代实机的遥控器。按时间排序，每项只触发一次，触发完后臂保持最后的姿态一直跑。
 * 接自己的算法时改这里，Arm 不用动。主臂第 0 个关节是 joint2。
 * 从 t=0 就使能，不留保护状态，否则 6 kg 的臂会在零力矩下先塌一截。 */
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
        /* 先让所有关节闭环在当前角度，再给 joint2 一个目标 */
        for (int i = 0; i < kJointCount; ++i)
        {
            arm.SetHostTarget(i, arm.observer().joint.deg[i], true);
        }
        arm.SetHostTarget(0, 20.0f, true);
        std::printf("[input] t=%.2f -> HOST (所有关节闭环), joint2 target +20 deg\n", time_s);
    }
}

void InputGripClose(Arm &arm, double time_s)
{
    arm.SetGripperTarget(0.008f);
    std::printf("[input] t=%.2f -> gripper close to 8 mm\n", time_s);
}

void InputReturn(Arm &arm, double time_s)
{
    if (arm.state() == Arm::State::Host)
    {
        arm.SetHostTarget(0, 0.0f, true);
        std::printf("[input] t=%.2f -> joint2 target 0 deg\n", time_s);
    }
}

void InputGripOpen(Arm &arm, double time_s)
{
    arm.SetGripperTarget(0.0f);
    std::printf("[input] t=%.2f -> gripper open\n", time_s);
}

constexpr InputEvent kInputEvents[] = {
    {0.0, InputEnable},
    {0.5, InputHost},
    {2.0, InputGripClose},
    {3.5, InputReturn},
    {5.0, InputGripOpen},
};

/*--------------------------------------监视--------------------------------------*/
void PrintStatus(const MujocoWorld &world, const Arm &arm)
{
    const Arm::Observer &obs = arm.observer();
    const Arm::Output &out = arm.control().output;
    const auto tcp =
        world.SitePosition(ModelName(world, "dh8")).value_or(std::array<double, 3>{});
    float tau_max = std::fabs(out.waist_nm);

    for (int i = 0; i < kJointCount; ++i)
    {
        tau_max = std::fmax(tau_max, std::fabs(out.joint_nm[i]));
    }
    std::printf("t=%6.2f  %-9s  waist=%6.2f  q=[%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f]  "
                "grip=%5.1f mm  tcp=[%6.3f %6.3f %6.3f]  |tau|max=%6.3f  warn=%d\n",
                world.time(), ToString(arm.state()), obs.waist.deg,
                obs.joint.deg[0], obs.joint.deg[1], obs.joint.deg[2], obs.joint.deg[3],
                obs.joint.deg[4], obs.joint.deg[5], obs.joint.deg[6],
                obs.gripper.m * 1000.0f, tcp[0], tcp[1], tcp[2], tau_max, world.warning_count());
}

void PrintUsage(const char *prog)
{
    std::printf("用法: %s [模型.xml] [秒数] [选项]\n"
                "  默认模型 %s\n"
                "  默认场地 %s\n"
                "  秒数         不给就一直跑，Ctrl-C 或关窗口退出\n"
                "  --fast       不按实时节拍 sleep\n"
                "  --host       开伪串口 %s 等 Host/armhost 连接，由上位机给主臂目标\n"
                "  --no-viewer  不开 MuJoCo 窗口\n"
                "  --full-viewer\n"
                "               使用与物理模型相同的查看器模型（自定义场地时可直接看到完整模型）\n"
                "  --arena XML  合并指定场地（默认普通 MuJoCo 地面）\n"
                "  --no-arena   只加载机械臂模型\n"
                "  --arm-position X Y Z\n"
                "               机械臂在场地世界系的位置，默认 5 -5 0.01 m\n"
                "  --exchange-position X Y Z\n"
                "               兑换站装配坐标系原点 O 的世界位置，默认 4.05 -5 0 m\n"
                "  --no-exchange\n"
                "               不加载兑换站\n"
                "  --exchange-free\n"
                "               启动时释放兑换站关节（可在窗口里手动拖动；Host 点“复位”重新接管）\n",
                prog, ARMSIM_DEFAULT_XML,
                ARMSIM_DEFAULT_ARENA[0] == '\0' ? "未配置" : ARMSIM_DEFAULT_ARENA,
                kHostLinkPath);
}

}  // namespace

int main(int argc, char **argv)
{
    const char *xml_path = ARMSIM_DEFAULT_XML;
    std::string arena_path = ARMSIM_DEFAULT_ARENA;
    std::string exchange_xml_path = ARMSIM_DEFAULT_EXCHANGE;
    std::array<double, 3> arm_position = kDefaultArmPosition;
    std::array<double, 3> exchange_position = kDefaultExchangePosition;
    double duration_s = std::numeric_limits<double>::infinity();
    bool realtime = true;
    bool host_mode = false;
    bool use_viewer = true;
    bool full_viewer = false;
    bool use_exchange = true;
    bool exchange_free = false;
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
        else if (std::strcmp(argv[i], "--no-viewer") == 0)
        {
            use_viewer = false;
        }
        else if (std::strcmp(argv[i], "--full-viewer") == 0)
        {
            full_viewer = true;
        }
        else if (std::strcmp(argv[i], "--arena") == 0)
        {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "--arena 需要一个 MJCF 路径\n");
                return 2;
            }
            arena_path = argv[++i];
        }
        else if (std::strcmp(argv[i], "--no-arena") == 0)
        {
            arena_path.clear();
        }
        else if (std::strcmp(argv[i], "--no-exchange") == 0)
        {
            use_exchange = false;
        }
        else if (std::strcmp(argv[i], "--exchange-free") == 0)
        {
            exchange_free = true;
        }
        else if (std::strcmp(argv[i], "--arm-position") == 0)
        {
            if (i + 3 >= argc)
            {
                std::fprintf(stderr, "--arm-position 需要 X Y Z 三个数\n");
                return 2;
            }
            for (int axis = 0; axis < 3; ++axis)
            {
                const char *text = argv[++i];
                char *end = nullptr;
                arm_position[axis] = std::strtod(text, &end);
                if (end == text || *end != '\0' || !std::isfinite(arm_position[axis]))
                {
                    std::fprintf(stderr, "无效的机械臂位置 %s\n", text);
                    return 2;
                }
            }
        }
        else if (std::strcmp(argv[i], "--exchange-position") == 0)
        {
            if (i + 3 >= argc)
            {
                std::fprintf(stderr, "--exchange-position 需要 X Y Z 三个数\n");
                return 2;
            }
            for (int axis = 0; axis < 3; ++axis)
            {
                const char *text = argv[++i];
                char *end = nullptr;
                exchange_position[axis] = std::strtod(text, &end);
                if (end == text || *end != '\0' || !std::isfinite(exchange_position[axis]))
                {
                    std::fprintf(stderr, "无效的兑换站位置 %s\n", text);
                    return 2;
                }
            }
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
        }
    }
    if (host_mode)
    {
        realtime = true;   // 上位机是真实时间，仿真不能跑快
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    try
    {
        /* 构造：物理世界 -> 机械臂 ->（可选）上位机链路和协议桥 */
        MujocoWorld::Config world_config;
        world_config.xml_path = xml_path;
        world_config.arena_path = arena_path;
        world_config.view_arena_path =
            arena_path.empty() || full_viewer ? "" : ARMSIM_DEFAULT_VIEW_ARENA;
        world_config.model_prefix = arena_path.empty() ? "" : kArenaModelPrefix;
        world_config.model_position = arm_position;
        const bool station_enabled = use_exchange && !arena_path.empty() &&
                                     !exchange_xml_path.empty();
        world_config.exchange_xml_path = station_enabled ? exchange_xml_path : "";
        world_config.exchange_position = exchange_position;
        world_config.exchange_prefix = kExchangeModelPrefix;
        world_config.joint_armature = kJointArmature;
        world_config.joint_damping = kJointDamping;
        world_config.joint_frictionloss = kJointFrictionLoss;

        MujocoWorld world(world_config);
        Arm arm(world, MakeArmConfig(world));
        std::unique_ptr<ExchangeStation> exchange;
        if (station_enabled)
        {
            exchange = std::make_unique<ExchangeStation>(world, kExchangeModelPrefix);
            if (exchange_free)
            {
                ExchangeStation::Command free_cmd;
                free_cmd.action = ExchangeStation::Action::Free;
                exchange->HandleCommand(free_cmd);
                std::printf("[system] exchange: 关节已释放，可在窗口里拖动\n");
            }
            const auto pose = exchange->GetSnapshot().pose;
            std::printf("[system] exchange: %s @ O=[%.3f %.3f %.3f], "
                        "core pose=[%.0f %.0f %.0f] mm\n",
                        exchange_xml_path.c_str(), exchange_position[0], exchange_position[1],
                        exchange_position[2], pose.x_m * 1000.0, pose.y_m * 1000.0,
                        pose.z_m * 1000.0);
        }

        std::unique_ptr<HostLink> link;
        std::unique_ptr<Host> host;
        if (host_mode)
        {
            link = std::make_unique<HostLink>(kHostLinkPath);
            host = std::make_unique<Host>(arm, *link, kHostStateDivider, exchange.get());
            /* 进入 HOST 时主臂没有任何关节闭环，只有重力前馈托着，等上位机发 SET_HOST_TARGET；
             * 腰部和夹爪由 Arm 自己保持 */
            arm.SetState(Arm::State::Host);
            std::printf("[system] host link: %s -> %s  (921600 8N1，Host/armhost 直接连这个口)\n",
                        link->link_path().c_str(), link->device_path().c_str());
        }

#ifdef ARMSIM_HAS_VIEWER
        std::unique_ptr<Viewer> viewer;
        if (use_viewer)
        {
            try
            {
                viewer = std::make_unique<Viewer>(world);
            }
            catch (const std::exception &e)
            {
                std::fprintf(stderr, "[system] 窗口打不开，改为无窗口运行: %s\n", e.what());
            }
        }
#else
        if (use_viewer)
        {
            std::printf("[system] 这个构建没有 simulate / GLFW，无窗口运行（apt install libglfw3-dev 后重新构建）\n");
        }
#endif

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
        if (arena_path.empty())
        {
            std::printf("[system] model=%s, arena=none, dt=%.4f s, Ctrl-C 退出\n", xml_path, dt);
        }
        else
        {
            std::printf("[system] model=%s, arena=%s, position=[%.3f %.3f %.3f], "
                        "prefix=%s, dt=%.4f s, Ctrl-C 退出\n",
                        xml_path, arena_path.c_str(), arm_position[0], arm_position[1],
                        arm_position[2], world.model_prefix().c_str(), dt);
        }

        /* 步进仿真：输入 -> 臂控制 -> 上位机收发 -> 物理步进 -> 监视输出。
         * 有窗口时这个循环跑在副线程，窗口占主线程；推进物理期间持有查看器的锁，睡眠时放开 */
        auto simulate = [&]() {
            const auto wall_start = std::chrono::steady_clock::now();
            size_t next_event = 0;
            int step = 0;

            while (world.time() < duration_s && !g_stop)
            {
#ifdef ARMSIM_HAS_VIEWER
                if (viewer && viewer->closed())
                {
                    break;
                }
                std::unique_lock<std::recursive_mutex> lock;
                if (viewer)
                {
                    lock = viewer->Lock();
                }
#endif
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

                if (exchange)
                {
                    exchange->Step(dt);
                }
                world.Step();

                if (!host_mode && step % kMonitorEverySteps == 0)   // 状态由上位机看，终端不刷屏
                {
                    PrintStatus(world, arm);
                }
                ++step;

#ifdef ARMSIM_HAS_VIEWER
                if (viewer && step % kSyncEverySteps == 0)
                {
                    viewer->Sync();
                }
                if (lock.owns_lock())
                {
                    lock.unlock();
                }
#endif

                if (realtime)
                {
                    std::this_thread::sleep_until(wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                                   std::chrono::duration<double>(world.time())));
                }
            }
        };

#ifdef ARMSIM_HAS_VIEWER
        if (viewer)
        {
            viewer->Run(simulate);
        }
        else
#endif
        {
            simulate();
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
