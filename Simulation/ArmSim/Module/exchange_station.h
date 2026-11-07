#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "mujoco_world.h"

/* RMUC2027 科技核心 / 兑换站。
 *
 * 坐标系与规则手册 5.4.3 一致：O 为底座正面与地面交线中点，X 指入底座
 * 内部，Z 向上。科技核心位姿用 (x,y,z,theta,phi,alpha) 表示；前三个
 * 为米，后三个为弧度。装配顺序固定为：
 *   到位 -> 沿 X 负向插入 100 mm -> 沿 Z 正向抬起 100 mm
 *        -> 绕 P 轴抬升 90 deg -> 绕 Q 轴转到指定角 -> 确认
 * 确认点随难度变化：一级插入后、二级抬起后、三/四级 Q 轴到位后。
 * 确认成功后 2 s 开始回收（回到停放位）。
 *
 * 这个类只做“兑换站机构 + 仿真状态机”，不绕过 Host 协议直接碰 UI。
 * 所有动作都驱动 MJCF 里的 position actuator，物理仍由 MuJoCo 推进。
 */
class ExchangeStation {
public:
    enum class Action : uint8_t {
        None = 0,
        Reset = 1,
        Randomize = 2,
        MoveToPose = 3,
        Insert = 4,
        Lift = 5,
        RotateP = 6,
        RotateQ = 7,
        Confirm = 8,
        AutoDemo = 9,
        StopAuto = 10,
        Free = 11,   /* 撤掉所有驱动，关节可手动拖动；Reset 重新接管并回到停放位 */
    };

    enum class Phase : uint8_t {
        Parked = 0,
        MovingPose = 1,
        Ready = 2,
        Inserting = 3,
        Inserted = 4,
        Lifting = 5,
        Lifted = 6,
        RotatingP = 7,
        PRotated = 8,
        RotatingQ = 9,
        QReady = 10,
        Complete = 11,
        Resetting = 12,
    };

    struct Pose {
        double x_m = -0.200;
        double y_m = 0.200;
        double z_m = 0.800;
        double theta_rad = 0.0;
        double phi_rad = 1.5707963267948966;
        double alpha_rad = 0.0;
    };

    struct Command {
        Action action = Action::None;
        int difficulty = 1;
        double q_target_deg = 0.0;
        uint64_t seed = 0;
        Pose pose;
        bool use_pose = false;
    };

    struct Snapshot {
        Phase phase = Phase::Parked;
        int difficulty = 1;
        bool moving = false;
        bool auto_demo = false;
        bool free = false;
        double q_target_deg = 0.0;
        uint64_t seed = 0;
        Pose pose;
        Pose target_pose;
        double insert_m = 0.0;
        double lift_m = 0.0;
        double p_rad = 0.0;
        double q_rad = 0.0;
    };

    ExchangeStation(MujocoWorld &world, const std::string &prefix = "exchange/");
    ExchangeStation(const ExchangeStation &) = delete;
    ExchangeStation &operator=(const ExchangeStation &) = delete;

    /* 返回 false 表示当前阶段不允许该动作或参数越界。调用方负责回 BAD_PARAM。 */
    bool HandleCommand(const Command &command);
    void Step(double dt);
    Snapshot GetSnapshot() const;

    static const char *PhaseName(Phase phase);

private:
    static constexpr int kLevelJoints = 6;
    static constexpr int kJointCount = 9;
    /* 关节顺序：0-5 为位姿 x y z θ φ α，其后依次为上移、P 轴、Q 轴。
     * 没有“插入”关节：插入由机器人完成，这里只作为一个不产生运动的阶段。 */
    enum Joint { kJx, kJy, kJz, kJtheta, kJphi, kJalpha, kJlift, kJp, kJq };

    struct Motion {
        bool active = false;
        double start_s = 0.0;
        double duration_s = 0.0;
        double start = 0.0;
        double target = 0.0;
    };

    void BindJoints(const std::string &prefix);
    void UpdateActuators(const std::array<double, kJointCount> &q);
    void StartMotion(const std::array<double, kJointCount> &target, double duration_s,
                     Phase phase);
    bool MotionDone() const;
    bool TargetReached() const;
    void BeginRandomPose(uint64_t seed);
    bool BeginInsert();
    bool BeginLift();
    bool BeginRotateP();
    bool BeginRotateQ();
    bool BeginConfirm();
    Phase ConfirmPhase() const;
    void BeginReset();
    void SetFree(bool free);
    void AutoAdvance();
    std::array<double, kJointCount> CurrentQ() const;
    const std::array<double, kJointCount> &TargetQ() const { return target_q_; }

    MujocoWorld &world_;
    std::array<int, kJointCount> joint_;
    std::array<int, kJointCount> actuator_;
    std::array<double, kJointCount> target_q_{};
    std::array<Motion, kJointCount> motion_{};

    Phase phase_ = Phase::Parked;
    int difficulty_ = 1;
    double elapsed_s_ = 0.0;
    double q_target_deg_ = 0.0;
    uint64_t seed_ = 0;
    Pose pose_target_;
    bool auto_demo_ = false;
    bool pending_auto_start_ = false;
    bool free_ = false;
    std::string prefix_;
    double auto_next_s_ = 0.0;
    double complete_s_ = 0.0;
    double phase_started_s_ = 0.0;
    double phase_deadline_s_ = 0.0;
};

uint8_t ExchangePhaseCode(ExchangeStation::Phase phase);
const char *ToString(ExchangeStation::Phase phase);
