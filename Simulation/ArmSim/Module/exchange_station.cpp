#include "exchange_station.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

constexpr double kPoseMoveS = 2.0;
constexpr double kResetMoveS = 1.5;
constexpr double kInsertMoveS = 1.2;
constexpr double kLiftMoveS = 1.0;
constexpr double kRotatePMoveS = 1.0;
constexpr double kRotateQMoveS = 0.8;
constexpr double kRecycleDelayS = 2.0;      // 确认后到回收开始的时间
constexpr double kPositionToleranceM = 0.005;   // 滑台到位与确认共用，避免两处判据不一致
constexpr double kQToleranceDeg = 5.0;      // 规则：Q 轴 5° 精度

double SmoothStep(double t)
{
    t = std::clamp(t, 0.0, 1.0);
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

bool InRange(double value, double lo, double hi)
{
    return value >= lo && value <= hi;
}

uint64_t MakeSeed()
{
    const uint64_t clock = static_cast<uint64_t>(
        std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::random_device device;
    return clock ^ (static_cast<uint64_t>(device()) << 32) ^ device();
}

}  // namespace

ExchangeStation::ExchangeStation(MujocoWorld &world, const std::string &prefix)
    : world_(world), prefix_(prefix)
{
    BindJoints(prefix);
    target_q_ = CurrentQ();
    UpdateActuators(target_q_);
}

void ExchangeStation::BindJoints(const std::string &prefix)
{
    const std::array<std::string, kJointCount> joint_names = {
        "level_x", "level_y", "level_z", "level_theta", "level_phi", "level_alpha",
        "step3_slide", "p_axis", "q_axis",
    };
    const std::array<std::string, kJointCount> actuator_names = {
        "ctrl_level_x", "ctrl_level_y", "ctrl_level_z", "ctrl_level_theta",
        "ctrl_level_phi", "ctrl_level_alpha", "ctrl_m_axis",
        "ctrl_p_axis", "ctrl_q_axis",
    };

    for (int i = 0; i < kJointCount; ++i)
    {
        joint_[i] = world_.BindJoint(prefix + joint_names[i]);
        actuator_[i] = world_.BindActuator(prefix + actuator_names[i]);
    }
}

void ExchangeStation::UpdateActuators(const std::array<double, kJointCount> &q)
{
    for (int i = 0; i < kJointCount; ++i)
    {
        world_.SetActuatorControl(actuator_[i], q[i]);
    }
}

std::array<double, ExchangeStation::kJointCount> ExchangeStation::CurrentQ() const
{
    std::array<double, kJointCount> q{};
    for (int i = 0; i < kJointCount; ++i)
    {
        q[i] = world_.JointPosition(joint_[i]);
    }
    return q;
}

void ExchangeStation::StartMotion(const std::array<double, kJointCount> &target,
                                  double duration_s, Phase phase)
{
    const auto current = CurrentQ();
    for (int i = 0; i < kJointCount; ++i)
    {
        target_q_[i] = target[i];
        Motion &m = motion_[i];
        m.start_s = elapsed_s_;
        m.duration_s = duration_s;
        m.start = current[i];
        m.target = target[i];
        m.active = std::fabs(m.target - m.start) > 1e-9 && duration_s > 0.0;
    }
    phase_ = phase;
    phase_started_s_ = elapsed_s_;
    phase_deadline_s_ = elapsed_s_ + duration_s + 5.0;
    if (auto_demo_)
    {
        auto_next_s_ = elapsed_s_ + duration_s + 0.20;
    }
}

bool ExchangeStation::MotionDone() const
{
    return std::none_of(motion_.begin(), motion_.end(),
                        [](const Motion &m) { return m.active; });
}

bool ExchangeStation::TargetReached() const
{
    /* 所有关节都要回到各自目标附近，而不只是当前阶段在动的那个：P/Q 轴快速转动会把
     * 插入滑台带出去一小段，不等它回来就报“到位”会让后续确认判据被瞬态误伤。 */
    static constexpr std::array<double, kJointCount> kTolerance = {
        0.002, 0.002, 0.002,                                   // 位姿 x y z（m）
        2.0 * kDegToRad, 2.0 * kDegToRad, 2.0 * kDegToRad,     // θ φ α
        kPositionToleranceM,                                   // 上移
        3.0 * kDegToRad, kQToleranceDeg * kDegToRad,           // P / Q
    };
    const auto q = CurrentQ();
    for (int i = 0; i < kJointCount; ++i)
    {
        if (std::fabs(q[i] - target_q_[i]) > kTolerance[i])
        {
            return false;
        }
    }
    return true;
}

void ExchangeStation::Step(double dt)
{
    elapsed_s_ += std::max(0.0, dt);
    if (free_)
    {
        return;   // 驱动已撤掉，关节完全交给手
    }
    std::array<double, kJointCount> q = target_q_;

    for (int i = 0; i < kJointCount; ++i)
    {
        Motion &m = motion_[i];
        if (!m.active)
        {
            continue;
        }
        const double raw = m.duration_s > 0.0 ? (elapsed_s_ - m.start_s) / m.duration_s : 1.0;
        if (raw >= 1.0)
        {
            q[i] = m.target;
            m.active = false;
        }
        else
        {
            q[i] = m.start + (m.target - m.start) * SmoothStep(raw);
        }
    }
    UpdateActuators(q);

    if (!MotionDone() || (!TargetReached() && elapsed_s_ < phase_deadline_s_))
    {
        return;
    }

    switch (phase_)
    {
    case Phase::MovingPose:
    case Phase::Resetting:
        phase_ = phase_ == Phase::MovingPose ? Phase::Ready : Phase::Parked;
        break;
    case Phase::Inserting:
        phase_ = Phase::Inserted;
        break;
    case Phase::Lifting:
        phase_ = Phase::Lifted;
        break;
    case Phase::RotatingP:
        phase_ = Phase::PRotated;
        break;
    case Phase::RotatingQ:
        phase_ = Phase::QReady;
        break;
    case Phase::Complete:
        if (elapsed_s_ - complete_s_ >= kRecycleDelayS)
        {
            BeginReset();   // 规则：确认后能量单元被回收，科技核心回到初始位姿
        }
        return;
    default:
        break;
    }

    AutoAdvance();
}

bool ExchangeStation::HandleCommand(const Command &command)
{
    switch (command.action)
    {
    case Action::None:
        return true;
    case Action::StopAuto:
        auto_demo_ = false;
        pending_auto_start_ = false;
        return true;
    case Action::Free:
        SetFree(true);
        return true;
    case Action::Reset:
        auto_demo_ = false;
        pending_auto_start_ = false;
        if (free_)
        {
            SetFree(false);
        }
        BeginReset();
        return true;
    case Action::AutoDemo:
        if (free_)
        {
            return false;
        }
        difficulty_ = std::clamp(command.difficulty, 1, 4);
        auto_demo_ = true;
        pending_auto_start_ = true;
        if (phase_ == Phase::Parked || phase_ == Phase::Ready)
        {
            pending_auto_start_ = false;
            BeginRandomPose(command.seed);
        }
        else
        {
            BeginReset();
        }
        return true;
    default:
        break;
    }

    if (free_ || !MotionDone())
    {
        return false;
    }
    auto_demo_ = false;
    pending_auto_start_ = false;

    switch (command.action)
    {
    case Action::Randomize:
        if (phase_ != Phase::Parked && phase_ != Phase::Ready)
        {
            return false;
        }
        difficulty_ = std::clamp(command.difficulty, 1, 4);
        BeginRandomPose(command.seed);
        return true;
    case Action::MoveToPose:
        if (!command.use_pose || (phase_ != Phase::Parked && phase_ != Phase::Ready))
        {
            return false;
        }
        if (!InRange(command.pose.x_m, -0.100, 0.0) ||
            !InRange(command.pose.y_m, 0.100, 0.300) ||
            !InRange(command.pose.z_m, 0.500, 0.700) ||
            !InRange(command.pose.theta_rad, -kPi / 2.0, kPi / 2.0) ||
            !InRange(command.pose.phi_rad, 0.0, kPi / 2.0) ||
            !InRange(command.pose.alpha_rad, -kPi / 4.0, kPi / 4.0))
        {
            return false;
        }
        if (command.difficulty == 1)
        {
            const double tolerance = 0.5 * kDegToRad;
            if (std::fabs(command.pose.theta_rad) > tolerance ||
                std::fabs(command.pose.phi_rad - kPi / 2.0) > tolerance ||
                std::fabs(command.pose.alpha_rad) > tolerance)
            {
                return false;
            }
        }
        difficulty_ = std::clamp(command.difficulty, 1, 4);
        pose_target_ = command.pose;
        q_target_deg_ = std::clamp(command.q_target_deg, -90.0, 90.0);
        {
            auto target = CurrentQ();
            target[0] = pose_target_.x_m;
            target[1] = pose_target_.y_m;
            target[2] = pose_target_.z_m;
            target[3] = pose_target_.theta_rad;
            target[4] = pose_target_.phi_rad;
            target[5] = pose_target_.alpha_rad;
            target[kJlift] = 0.0;
            target[kJp] = 0.0;
            target[kJq] = 0.0;
            StartMotion(target, kPoseMoveS, Phase::MovingPose);
        }
        return true;
    case Action::Insert:
        return BeginInsert();
    case Action::Lift:
        return BeginLift();
    case Action::RotateP:
        return BeginRotateP();
    case Action::RotateQ:
        q_target_deg_ = std::clamp(command.q_target_deg, -90.0, 90.0);
        return BeginRotateQ();
    case Action::Confirm:
        return BeginConfirm();
    default:
        return false;
    }
}

void ExchangeStation::BeginRandomPose(uint64_t seed)
{
    seed_ = seed == 0 ? MakeSeed() : seed;
    std::mt19937_64 rng(seed_);
    difficulty_ = std::clamp(difficulty_, 1, 4);

    std::uniform_real_distribution<double> x_dist(-0.100, 0.0);
    std::uniform_real_distribution<double> y_dist(0.100, 0.300);
    std::uniform_real_distribution<double> z_dist(0.500, 0.700);
    pose_target_.x_m = x_dist(rng);
    pose_target_.y_m = y_dist(rng);
    pose_target_.z_m = z_dist(rng);

    if (difficulty_ == 1)
    {
        pose_target_.theta_rad = 0.0;
        pose_target_.phi_rad = kPi / 2.0;
        pose_target_.alpha_rad = 0.0;
    }
    else
    {
        std::uniform_real_distribution<double> theta_dist(-kPi / 2.0, kPi / 2.0);
        std::uniform_real_distribution<double> phi_dist(0.0, kPi / 2.0);
        std::uniform_real_distribution<double> alpha_dist(-kPi / 4.0, kPi / 4.0);
        pose_target_.theta_rad = theta_dist(rng);
        pose_target_.phi_rad = phi_dist(rng);
        pose_target_.alpha_rad = alpha_dist(rng);
    }
    if (difficulty_ >= 3)
    {
        std::uniform_int_distribution<int> q_dist(-18, 18);   // 规则：5° 精度，±90° 范围
        q_target_deg_ = 5.0 * q_dist(rng);
    }
    else
    {
        q_target_deg_ = 0.0;
    }

    auto target = CurrentQ();
    target[0] = pose_target_.x_m;
    target[1] = pose_target_.y_m;
    target[2] = pose_target_.z_m;
    target[3] = pose_target_.theta_rad;
    target[4] = pose_target_.phi_rad;
    target[5] = pose_target_.alpha_rad;
    target[kJlift] = 0.0;
    target[kJp] = 0.0;
    target[kJq] = 0.0;
    StartMotion(target, kPoseMoveS, Phase::MovingPose);
}

bool ExchangeStation::BeginInsert()
{
    if (phase_ != Phase::Ready)
    {
        return false;
    }
    /* 目标不变，StartMotion 里没有关节需要动，只是让阶段经过 Inserting 再到 Inserted。 */
    StartMotion(TargetQ(), kInsertMoveS, Phase::Inserting);
    return true;
}

bool ExchangeStation::BeginLift()
{
    if (phase_ != Phase::Inserted)
    {
        return false;
    }
    auto target = TargetQ();
    target[kJlift] = 0.100;
    StartMotion(target, kLiftMoveS, Phase::Lifting);
    return true;
}

bool ExchangeStation::BeginRotateP()
{
    if (phase_ != Phase::Lifted)
    {
        return false;
    }
    auto target = TargetQ();
    target[kJp] = kPi / 2.0;
    StartMotion(target, kRotatePMoveS, Phase::RotatingP);
    return true;
}

bool ExchangeStation::BeginRotateQ()
{
    if (phase_ != Phase::PRotated)
    {
        return false;
    }
    auto target = TargetQ();
    target[kJq] = q_target_deg_ * kDegToRad;
    StartMotion(target, kRotateQMoveS, Phase::RotatingQ);
    return true;
}

ExchangeStation::Phase ExchangeStation::ConfirmPhase() const
{
    /* 规则 5.4.3：一级完成步骤 2，二级完成 2、3，三/四级完成 2~5 后确认 */
    switch (difficulty_)
    {
    case 1: return Phase::Inserted;
    case 2: return Phase::Lifted;
    default: return Phase::QReady;
    }
}

bool ExchangeStation::BeginConfirm()
{
    if (phase_ != ConfirmPhase())
    {
        return false;
    }
    const auto q = CurrentQ();
    if (difficulty_ >= 2 && std::fabs(q[kJlift] - 0.100) > kPositionToleranceM)
    {
        return false;
    }
    if (difficulty_ >= 3 &&
        (std::fabs(q[kJp] - kPi / 2.0) > 3.0 * kDegToRad ||
         std::fabs(q[kJq] - q_target_deg_ * kDegToRad) > kQToleranceDeg * kDegToRad))
    {
        return false;
    }
    phase_ = Phase::Complete;
    complete_s_ = elapsed_s_;
    auto_demo_ = false;
    pending_auto_start_ = false;
    return true;
}

void ExchangeStation::SetFree(bool free)
{
    if (free == free_)
    {
        return;
    }
    free_ = free;
    for (int i = 0; i < kJointCount; ++i)
    {
        world_.SetActuatorEnabled(actuator_[i], !free);
    }
    world_.SetSubtreeFree(prefix_ + "E", free);
    if (free)
    {
        auto_demo_ = false;
        pending_auto_start_ = false;
        for (Motion &m : motion_)
        {
            m.active = false;
        }
        phase_ = Phase::Parked;
    }
    else
    {
        /* 重新接管：目标先设成当前位置，避免驱动一接上就把关节猛拉回去，
         * 随后 BeginReset 会平滑回到停放位。 */
        target_q_ = CurrentQ();
        UpdateActuators(target_q_);
    }
}

void ExchangeStation::BeginReset()
{
    auto target = TargetQ();
    target[0] = Pose{}.x_m;
    target[1] = Pose{}.y_m;
    target[2] = Pose{}.z_m;
    target[3] = Pose{}.theta_rad;
    target[4] = Pose{}.phi_rad;
    target[5] = Pose{}.alpha_rad;
    target[kJlift] = 0.0;
    target[kJp] = 0.0;
    target[kJq] = 0.0;
    StartMotion(target, kResetMoveS, Phase::Resetting);
}

void ExchangeStation::AutoAdvance()
{
    if (!auto_demo_ || elapsed_s_ < auto_next_s_)
    {
        return;
    }
    if (phase_ == Phase::Parked && pending_auto_start_)
    {
        pending_auto_start_ = false;
        BeginRandomPose(seed_);
        return;
    }
    if (phase_ == ConfirmPhase())
    {
        BeginConfirm();
        return;
    }
    switch (phase_)
    {
    case Phase::Ready:
        BeginInsert();
        break;
    case Phase::Inserted:
        BeginLift();
        break;
    case Phase::Lifted:
        BeginRotateP();
        break;
    case Phase::PRotated:
        BeginRotateQ();
        break;
    default:
        break;
    }
}

ExchangeStation::Snapshot ExchangeStation::GetSnapshot() const
{
    Snapshot out;
    const auto q = CurrentQ();
    out.phase = phase_;
    out.difficulty = difficulty_;
    out.moving = !MotionDone() || phase_ == Phase::MovingPose || phase_ == Phase::Inserting ||
                 phase_ == Phase::Lifting || phase_ == Phase::RotatingP ||
                 phase_ == Phase::RotatingQ || phase_ == Phase::Resetting;
    out.auto_demo = auto_demo_;
    out.free = free_;
    out.q_target_deg = q_target_deg_;
    out.seed = seed_;
    out.pose = {q[0], q[1], q[2], q[3], q[4], q[5]};
    out.target_pose = {target_q_[0], target_q_[1], target_q_[2],
                       target_q_[3], target_q_[4], target_q_[5]};
    /* 没有插入关节：按阶段给出步骤 2 的完成情况（-100 mm = 已插入），协议字段保持不变。 */
    out.insert_m = (phase_ == Phase::Parked || phase_ == Phase::MovingPose ||
                    phase_ == Phase::Ready || phase_ == Phase::Inserting ||
                    phase_ == Phase::Resetting) ? 0.0 : -0.100;
    out.lift_m = q[kJlift];
    out.p_rad = q[kJp];
    out.q_rad = q[kJq];
    return out;
}

const char *ExchangeStation::PhaseName(Phase phase)
{
    switch (phase)
    {
    case Phase::Parked: return "PARKED";
    case Phase::MovingPose: return "MOVING_POSE";
    case Phase::Ready: return "READY";
    case Phase::Inserting: return "INSERTING";
    case Phase::Inserted: return "INSERTED";
    case Phase::Lifting: return "LIFTING";
    case Phase::Lifted: return "LIFTED";
    case Phase::RotatingP: return "ROTATING_P";
    case Phase::PRotated: return "P_ROTATED";
    case Phase::RotatingQ: return "ROTATING_Q";
    case Phase::QReady: return "Q_READY";
    case Phase::Complete: return "COMPLETE";
    case Phase::Resetting: return "RESETTING";
    }
    return "UNKNOWN";
}

uint8_t ExchangePhaseCode(ExchangeStation::Phase phase)
{
    return static_cast<uint8_t>(phase);
}

const char *ToString(ExchangeStation::Phase phase)
{
    return ExchangeStation::PhaseName(phase);
}
