#include "arm.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace {

constexpr float kEncoderCpr = 65536.0f;

float Clamp(float value, float min_limit, float max_limit)
{
    return value > max_limit ? max_limit : (value < min_limit ? min_limit : value);
}

struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Quaternion QuaternionFromRotation(const Mat4 &T)
{
    Quaternion q;
    const double trace = T(0, 0) + T(1, 1) + T(2, 2);
    if (trace > 0.0)
    {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (T(2, 1) - T(1, 2)) / s;
        q.y = (T(0, 2) - T(2, 0)) / s;
        q.z = (T(1, 0) - T(0, 1)) / s;
    }
    else if (T(0, 0) > T(1, 1) && T(0, 0) > T(2, 2))
    {
        const double s = std::sqrt(1.0 + T(0, 0) - T(1, 1) - T(2, 2)) * 2.0;
        q.w = (T(2, 1) - T(1, 2)) / s;
        q.x = 0.25 * s;
        q.y = (T(0, 1) + T(1, 0)) / s;
        q.z = (T(0, 2) + T(2, 0)) / s;
    }
    else if (T(1, 1) > T(2, 2))
    {
        const double s = std::sqrt(1.0 + T(1, 1) - T(0, 0) - T(2, 2)) * 2.0;
        q.w = (T(0, 2) - T(2, 0)) / s;
        q.x = (T(0, 1) + T(1, 0)) / s;
        q.y = 0.25 * s;
        q.z = (T(1, 2) + T(2, 1)) / s;
    }
    else
    {
        const double s = std::sqrt(1.0 + T(2, 2) - T(0, 0) - T(1, 1)) * 2.0;
        q.w = (T(1, 0) - T(0, 1)) / s;
        q.x = (T(0, 2) + T(2, 0)) / s;
        q.y = (T(1, 2) + T(2, 1)) / s;
        q.z = 0.25 * s;
    }
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm > 1e-12)
    {
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }
    if (q.w < 0.0)
    {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }
    return q;
}

double QuaternionDot(const Quaternion &a, const Quaternion &b)
{
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

Quaternion Slerp(Quaternion a, Quaternion b, double t)
{
    double dot = QuaternionDot(a, b);
    if (dot < 0.0)
    {
        b.w = -b.w;
        b.x = -b.x;
        b.y = -b.y;
        b.z = -b.z;
        dot = -dot;
    }
    double wa = 1.0 - t;
    double wb = t;
    if (dot < 0.9995)
    {
        const double theta = std::acos(std::fmax(-1.0, std::fmin(1.0, dot)));
        const double sin_theta = std::sin(theta);
        wa = std::sin((1.0 - t) * theta) / sin_theta;
        wb = std::sin(t * theta) / sin_theta;
    }
    Quaternion q{wa * a.w + wb * b.w, wa * a.x + wb * b.x,
                 wa * a.y + wb * b.y, wa * a.z + wb * b.z};
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm > 1e-12)
    {
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }
    return q;
}

Mat4 RotationFromQuaternion(const Quaternion &q)
{
    Mat4 T = Mat4::Identity();
    const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    T(0, 0) = 1.0 - 2.0 * (yy + zz);
    T(0, 1) = 2.0 * (xy - wz);
    T(0, 2) = 2.0 * (xz + wy);
    T(1, 0) = 2.0 * (xy + wz);
    T(1, 1) = 1.0 - 2.0 * (xx + zz);
    T(1, 2) = 2.0 * (yz - wx);
    T(2, 0) = 2.0 * (xz - wy);
    T(2, 1) = 2.0 * (yz + wx);
    T(2, 2) = 1.0 - 2.0 * (xx + yy);
    return T;
}

Mat4 InterpolatePose(const Mat4 &start, const Mat4 &goal, double t)
{
    if (t <= 0.0)
    {
        return start;
    }
    if (t >= 1.0)
    {
        return goal;
    }
    Mat4 out = RotationFromQuaternion(
        Slerp(QuaternionFromRotation(start), QuaternionFromRotation(goal), t));
    for (int r = 0; r < 3; ++r)
    {
        out(r, 3) = (1.0 - t) * start(r, 3) + t * goal(r, 3);
    }
    return out;
}

double RotationDistanceRad(const Mat4 &a, const Mat4 &b)
{
    const double dot = std::fabs(QuaternionDot(QuaternionFromRotation(a), QuaternionFromRotation(b)));
    return 2.0 * std::acos(std::fmax(-1.0, std::fmin(1.0, dot)));
}

float WrapPi(float angle_rad)
{
    angle_rad = std::fmod(angle_rad + kPi, kTwoPi);
    if (angle_rad < 0.0f)
    {
        angle_rad += kTwoPi;
    }
    return angle_rad - kPi;
}

constexpr float kAutoPsiMinDeg = -180.0f;
constexpr float kAutoPsiMaxDeg = 180.0f;
constexpr float kAutoPsiStepDeg = 2.0f;
constexpr float kAutoPsiCostTieEps = 1e-6f;
constexpr double kNonSingularMinSin = 0.02;   // 与 Host 可达区间一致：约 1.15°
constexpr int kAutoPsiSampleCount =
    static_cast<int>((kAutoPsiMaxDeg - kAutoPsiMinDeg) / kAutoPsiStepDeg) + 1;

bool MapIkResultToJointTargets(const IkResult &solved, const Arm::Observer &observer,
                               const Arm::Config &config, const JointArray &reference_deg,
                               JointArray &out_deg)
{
    for (int i = 0; i < kJointCount; ++i)
    {
        const size_t index = static_cast<size_t>(i);
        const float multi_deg = observer.joint.deg[index];
        const float abs_deg = observer.joint.abs_deg[index];
        const float delta_deg = WrapPi((solved.q_deg[index] - abs_deg) * kDegToRad) * kRadToDeg;
        const float base_deg = multi_deg + delta_deg;
        const float limit_min_deg = multi_deg + (config.limit[index].min_deg - abs_deg);
        const float limit_max_deg = multi_deg + (config.limit[index].max_deg - abs_deg);
        float best_deg = 0.0f;
        float best_error = 0.0f;
        bool found = false;

        if (!(limit_min_deg < limit_max_deg))
        {
            return false;
        }
        for (int turns = -2; turns <= 2; ++turns)
        {
            const float candidate_deg = base_deg + static_cast<float>(turns) * 360.0f;

            if (candidate_deg < limit_min_deg || candidate_deg > limit_max_deg)
            {
                continue;
            }
            const float error_deg = std::fabs(candidate_deg - reference_deg[index]);
            if (!found || error_deg < best_error)
            {
                found = true;
                best_deg = candidate_deg;
                best_error = error_deg;
            }
        }
        if (!found)
        {
            return false;
        }
        out_deg[index] = best_deg;
    }
    return true;
}

bool IsIkWellConditioned(const SrsParam &srs, const IkResult &solved)
{
    if (solved.status != IkStatus::Ok || solved.shoulder_singular || solved.wrist_singular)
    {
        return false;
    }
    for (int joint : {1, 5})
    {
        const double theta = static_cast<double>(solved.q_deg[static_cast<size_t>(joint)]) * kDegToRad +
                             srs.theta_offset_rad[static_cast<size_t>(joint)];
        if (std::fabs(std::sin(theta)) < kNonSingularMinSin)
        {
            return false;
        }
    }
    return true;
}

Arm::IkTargetResult SelectAutoPsi(const SrsParam &srs, const Mat4 &target, const Branch &branch,
                                  const Arm::Observer &observer, const Arm::Config &config,
                                  const JointArray &reference_deg, const IkCostWeights &weights,
                                  float preferred_psi_deg, float &psi_deg, IkResult &solved,
                                  JointArray &joint_target_deg, float &cost)
{
    bool found = false;
    bool saw_ik_solution = false;
    bool saw_ref_degenerate = false;
    IkResult best_solution;
    JointArray best_joint_target{};
    float best_psi_deg = 0.0f;
    float best_cost = std::numeric_limits<float>::infinity();

    auto consider_psi = [&](float sample_psi) {
        const IkResult candidate = SolveIk(srs, target, branch, sample_psi);

        if (candidate.status == IkStatus::RefDegenerate)
        {
            saw_ref_degenerate = true;
            return;
        }
        if (candidate.status != IkStatus::Ok)
        {
            return;
        }
        saw_ik_solution = true;
        if (!IsIkWellConditioned(srs, candidate))
        {
            return;
        }

        JointArray candidate_target{};
        if (!MapIkResultToJointTargets(candidate, observer, config, reference_deg, candidate_target))
        {
            return;
        }
        const float candidate_cost =
            ComputeJointMotionCost(candidate_target, reference_deg, weights);
        if (!std::isfinite(candidate_cost))
        {
            return;
        }
        if (!found || candidate_cost < best_cost - kAutoPsiCostTieEps)
        {
            found = true;
            best_solution = candidate;
            best_joint_target = candidate_target;
            best_psi_deg = sample_psi;
            best_cost = candidate_cost;
        }
    };

    if (std::isfinite(preferred_psi_deg))
    {
        consider_psi(preferred_psi_deg);
    }
    for (int sample = 0; sample < kAutoPsiSampleCount; ++sample)
    {
        const float sample_psi = kAutoPsiMinDeg + static_cast<float>(sample) * kAutoPsiStepDeg;
        consider_psi(sample_psi);
    }

    if (!found)
    {
        return saw_ik_solution ? Arm::IkTargetResult::OutOfLimit
                               : (saw_ref_degenerate ? Arm::IkTargetResult::RefDegenerate
                                                     : Arm::IkTargetResult::OutOfReach);
    }
    psi_deg = best_psi_deg;
    solved = best_solution;
    joint_target_deg = best_joint_target;
    cost = best_cost;
    return Arm::IkTargetResult::Ok;
}

}  // namespace

const char *ToString(Arm::State state)
{
    static const char *const kNames[] = {"PROTECTED", "ENABLE", "FEEDBACK", "MASTER",
                                         "AUTO", "SERVANT", "HOST", "HOST_IK"};
    const auto index = static_cast<size_t>(state);

    return index < sizeof(kNames) / sizeof(kNames[0]) ? kNames[index] : "?";
}

/*--------------------------------------构造--------------------------------------*/
Arm::Hardware::Hardware(MujocoWorld &world, const Config &config)
    : waist(world, config.waist_joint, config.joint_effort_limit)
{
    arm.reserve(kJointCount);
    for (int i = 0; i < kJointCount; ++i)
    {
        arm.emplace_back(world, config.joint_name[i], config.joint_effort_limit);
    }
    finger.reserve(config.gripper.finger_joint.size());
    for (const std::string &name : config.gripper.finger_joint)
    {
        finger.emplace_back(world, name, config.gripper.finger_effort_limit);
    }
}

Arm::Control::Control(const Config &config)
    : waist_pid(config.waist_gain), gripper_pid(config.gripper.gain)
{
    for (int i = 0; i < kJointCount; ++i)
    {
        joint_pid[i] = Pid(config.gain[i]);
    }
}

Arm::Arm(MujocoWorld &world, const Config &config)
    : config_(config),
      hardware_(world, config),
      gravity_(config.gravity, config.waist_mount, config.dh),
      srs_valid_(MakeSrsParam(config.dh, srs_)),
      control_(config)
{
    status_.state = config.init_state;
    target_.waist_deg = config.waist_hold_deg;
    target_.gripper_m = config.gripper.open_m;
    target_.host.align_pending = config.init_state == State::Host || config.init_state == State::HostIk;
}

/*--------------------------------------状态--------------------------------------*/
bool Arm::StateAllowed(State state) const
{
    switch (state)
    {
    case State::Protected:
    case State::Enable:
    case State::Host:
    case State::HostIk:
        return true;
    case State::Feedback:
    case State::Master:
        return config_.role == Role::Master;
    case State::Auto:
    case State::Servant:
        return config_.role == Role::Servant;
    }
    return false;
}

bool Arm::SetState(State state)
{
    if (!StateAllowed(state))
    {
        return false;
    }
    if (state == status_.state)
    {
        return true;
    }

    if (state == State::Protected)
    {
        status_.enabled = false;
    }
    status_.state = state;
    target_.align_pending = true;
    target_.host.align_pending = state == State::Host || state == State::HostIk;
    target_.host.closed_loop_mask = 0;
    target_.host.valid_mask = 0;
    if (state != State::HostIk)
    {
        target_.ik.active = false;
        target_.ik.planning = false;
        target_.ik.plan_ready = false;
        target_.ik.trajectory.clear();
        target_.ik.trajectory_index = 0;
    }
    return true;
}

/*--------------------------------------观测--------------------------------------*/
/* 执行器读数 -> 关节量（按 direction 和零点折算）-> 正运动学 -> TCP 位姿、臂角、分支 */
void Arm::Observe()
{
    Observer &obs = observer_;
    JointObs &joint = obs.joint;

    ++status_.cycle_count;
    for (int i = 0; i < kJointCount; ++i)
    {
        const JointActuator &actuator = hardware_.arm[i];
        const float direction = config_.direction[i] < 0 ? -1.0f : 1.0f;
        const float angle_rad = static_cast<float>(actuator.position()) - config_.zero_point_rad[i];
        const float abs_rad = WrapPi(angle_rad);

        joint.deg[i] = direction * angle_rad * kRadToDeg;
        joint.abs_deg[i] = direction * abs_rad * kRadToDeg;
        joint.vel_dps[i] = static_cast<float>(actuator.velocity()) * kRadToDeg * direction;
        joint.torque_nm[i] = static_cast<float>(actuator.effort()) * direction;
        joint.encoder[i] = static_cast<uint16_t>(
            static_cast<int32_t>(std::lround(abs_rad / kTwoPi * kEncoderCpr)) &
            static_cast<int32_t>(kEncoderCpr - 1.0f));
    }

    const FrameArray frames = ForwardKinematicsFrames(config_.dh, joint.deg);

    obs.pose = PoseFromTransform(frames.back() * config_.tcp);
    if (srs_valid_)
    {
        obs.posture.arm_angle = ComputeArmAngle(srs_, frames);
        obs.posture.branch = ComputeBranch(config_.dh, joint.deg);
    }

    obs.waist.deg = static_cast<float>(hardware_.waist.position()) * kRadToDeg;
    obs.waist.torque_nm = static_cast<float>(hardware_.waist.effort());
    obs.gripper.m = static_cast<float>(0.5 * (hardware_.finger[0].position() + hardware_.finger[1].position()));
    obs.gripper.force_n = static_cast<float>(hardware_.finger[0].effort());
}

/*--------------------------------------目标--------------------------------------*/
/* 限位映射到当前圈（差值不做环绕），目标对齐当前角，PID 清零 */
void Arm::AlignTarget()
{
    for (int i = 0; i < kJointCount; ++i)
    {
        JointTarget &target = target_.joint[i];
        const JointLimit &limit = config_.limit[i];
        const float multi_deg = observer_.joint.deg[i];
        const float abs_deg = observer_.joint.abs_deg[i];

        target.limit_min_deg = multi_deg + (limit.min_deg - abs_deg);
        target.limit_max_deg = multi_deg + (limit.max_deg - abs_deg);
        target.deg = Clamp(multi_deg, target.limit_min_deg, target.limit_max_deg);
        control_.output.joint_nm[i] = 0.0f;
        control_.joint_pid[i].Reset();
    }
    target_.align_pending = false;
}

/* HOST 的对齐：在 AlignTarget 之上再清掉上位机的目标和闭环掩码 */
void Arm::AlignHost()
{
    AlignTarget();
    target_.host.closed_loop_mask = 0;
    target_.host.valid_mask = 0;
    target_.host.align_pending = false;
}

void Arm::UpdateTarget()
{
    if ((status_.state == State::Host || status_.state == State::HostIk) && target_.host.align_pending)
    {
        AlignHost();
    }
    else if (status_.state == State::Enable && target_.align_pending)
    {
        AlignTarget();
    }

    if (status_.state == State::HostIk && target_.ik.active && target_.ik.method == IkMethod::Numeric)
    {
        StepNumericIk();
    }

    for (int i = 0; i < kJointCount; ++i)
    {
        JointTarget &target = target_.joint[i];

        switch (status_.state)
        {
        case State::Enable:   // 目标每周期跟随观测，靠重力前馈托住
            target.deg = Clamp(observer_.joint.deg[i], target.limit_min_deg, target.limit_max_deg);
            break;
        case State::Host:     // 目标由 SetHostTarget 写入，这里只做限位
        case State::HostIk:   // 目标由 SetIkTarget 写入，这里只做限位
            target.deg = Clamp(target.deg, target.limit_min_deg, target.limit_max_deg);
            break;
        default:
            target.deg = observer_.joint.deg[i];
            break;
        }
    }
}

Arm::HostTargetResult Arm::SetHostTarget(int joint, float target_deg, bool closed_loop,
                                         bool multi_turn)
{
    if (joint < 0 || joint >= kJointCount)
    {
        return HostTargetResult::BadTarget;
    }
    if (status_.state != State::Host)
    {
        return HostTargetResult::WrongState;
    }
    if (!std::isfinite(target_deg))
    {
        return HostTargetResult::BadTarget;
    }

    /* 第一次 HOST 控制前可能先收到目标：先锁存限位，再写目标 */
    if (target_.host.align_pending)
    {
        AlignHost();
    }

    JointTarget &joint_target = target_.joint[joint];
    if (multi_turn)
    {
        if (!(target_deg >= joint_target.limit_min_deg &&
              target_deg <= joint_target.limit_max_deg))
        {
            return HostTargetResult::BadTarget;
        }
        joint_target.deg = target_deg;
    }
    else
    {
        const JointLimit &limit = config_.limit[joint];
        if (!(target_deg >= limit.min_deg && target_deg <= limit.max_deg))
        {
            return HostTargetResult::BadTarget;
        }
        const float delta_deg = target_deg - observer_.joint.abs_deg[joint];
        joint_target.deg = Clamp(observer_.joint.deg[joint] + delta_deg,
                                 joint_target.limit_min_deg, joint_target.limit_max_deg);
    }

    HostTarget &host = target_.host;
    const uint8_t bit = static_cast<uint8_t>(1U << joint);
    const bool was_closed_loop = (host.closed_loop_mask & bit) != 0U;

    host.valid_mask |= bit;
    if (closed_loop)
    {
        host.closed_loop_mask |= bit;
    }
    else
    {
        host.closed_loop_mask &= static_cast<uint8_t>(~bit);
    }
    if (closed_loop != was_closed_loop)
    {
        control_.joint_pid[joint].Reset();
    }
    return HostTargetResult::Ok;
}

Arm::IkTargetResult Arm::SetIkTarget(const Mat4 &target, const Branch &branch, IkPsiMode psi_mode,
                                     float psi_deg, const IkCostWeights &cost_weights, IkMethod method)
{
    if (status_.state != State::Host && status_.state != State::HostIk)
    {
        return IkTargetResult::WrongState;
    }
    /* 协议和界面的 target 都是 TCP；S-R-S 解析/数值逆解在 link8 几何上计算。 */
    const Mat4 link_target = target * RigidInverse(config_.tcp);
    if (method == IkMethod::Numeric)
    {
        return SetNumericIkTarget(link_target, cost_weights);
    }
    if (method != IkMethod::Analytic)
    {
        return IkTargetResult::BadParam;
    }
    if (!srs_valid_ || branch.valid == false)
    {
        return IkTargetResult::BadBranch;
    }
    if (psi_mode != IkPsiMode::Manual && psi_mode != IkPsiMode::Auto)
    {
        return IkTargetResult::BadParam;
    }
    if (psi_mode == IkPsiMode::Manual && !std::isfinite(psi_deg))
    {
        return IkTargetResult::BadParam;
    }
    if (psi_mode == IkPsiMode::Auto && !IsValidIkCostWeights(cost_weights))
    {
        return IkTargetResult::BadParam;
    }

    /* 进入前先对齐一次，保证当前分支、限位所在圈和目标角都以同一组观测为基准 */
    if (target_.align_pending || target_.host.align_pending)
    {
        AlignHost();
    }

    if (status_.state == State::Host)
    {
        if (!observer_.posture.branch.valid || branch.index() != observer_.posture.branch.index())
        {
            return IkTargetResult::BadBranch;
        }
    }
    else if (target_.ik.active && target_.ik.method == IkMethod::Numeric)
    {
        /* 数值解切回解析解：解析解一步跳到目标，分支必须和当前实际构型一致 */
        if (!observer_.posture.branch.valid || branch.index() != observer_.posture.branch.index())
        {
            return IkTargetResult::BadBranch;
        }
    }
    else if (target_.ik.active && branch.index() != target_.ik.branch.index())
    {
        /* 逆解闭环期间不允许通过改分支跳变；先退出 IK，等实际构型进入新分支后再进入 */
        return IkTargetResult::BadBranch;
    }

    JointArray previous_target_deg{};
    for (int i = 0; i < kJointCount; ++i)
    {
        previous_target_deg[static_cast<size_t>(i)] = target_.joint[i].deg;
    }

    IkResult solved;
    std::array<float, kJointCount> next_target{};
    float cost = 0.0f;

    if (psi_mode == IkPsiMode::Auto)
    {
        const float preferred_psi_deg = target_.ik.active ? target_.ik.psi_deg
                                                          : observer_.posture.arm_angle.psi_deg;
        const IkTargetResult auto_result = SelectAutoPsi(
            srs_, link_target, branch, observer_, config_, previous_target_deg, cost_weights,
            preferred_psi_deg, psi_deg, solved, next_target, cost);
        if (auto_result != IkTargetResult::Ok)
        {
            return auto_result;
        }
    }
    else
    {
        solved = SolveIk(srs_, link_target, branch, psi_deg);

        if (solved.status == IkStatus::OutOfReach)
        {
            return IkTargetResult::OutOfReach;
        }
        if (solved.status == IkStatus::RefDegenerate)
        {
            return IkTargetResult::RefDegenerate;
        }
        if (!MapIkResultToJointTargets(solved, observer_, config_, previous_target_deg, next_target))
        {
            return IkTargetResult::OutOfLimit;
        }
        cost = ComputeJointMotionCost(next_target, previous_target_deg, cost_weights);
    }

    if (status_.state == State::Host)
    {
        if (!SetState(State::HostIk))
        {
            return IkTargetResult::WrongState;
        }
        AlignHost();
    }

    for (int i = 0; i < kJointCount; ++i)
    {
        target_.joint[i].deg = next_target[i];
    }
    target_.host.valid_mask = 0xFF;
    target_.host.closed_loop_mask = 0xFF;
    target_.host.align_pending = false;
    target_.align_pending = false;
    target_.ik.active = true;
    target_.ik.method = IkMethod::Analytic;
    target_.ik.limit_mask = 0;
    target_.ik.step_limited = false;
    target_.ik.converged = false;
    target_.ik.stall_cycles = 0;
    target_.ik.pose = link_target;
    target_.ik.psi_deg = psi_deg;
    target_.ik.psi_mode = psi_mode;
    target_.ik.cost_weights = cost_weights;
    target_.ik.cost = cost;
    target_.ik.branch = branch;
    target_.ik.status = solved.status;
    target_.ik.shoulder_singular = solved.shoulder_singular;
    target_.ik.wrist_singular = solved.wrist_singular;
    return IkTargetResult::Ok;
}

/* 数值解先算后跑：收到目标后只启动轨迹规划，保持当前关节目标不动。后续控制周期分批计算
 * 完整关节轨迹，算完后再按规划点逐周期执行；规划失败不会产生运动。 */
Arm::IkTargetResult Arm::SetNumericIkTarget(const Mat4 &target, const IkCostWeights &cost_weights)
{
    if (!IsValidIkCostWeights(cost_weights) || !IsValidNumericIkParam(config_.ik_numeric))
    {
        return IkTargetResult::BadParam;
    }
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            if (!std::isfinite(target(r, c)))
            {
                return IkTargetResult::BadParam;
            }
        }
    }

    if (status_.state == State::Host)
    {
        if (!SetState(State::HostIk))
        {
            return IkTargetResult::WrongState;
        }
    }
    /* 每次新数值规划都从实测关节重新对齐。若直接使用上一轮目标，轨迹完成后
     * 机构因静差/限幅没有跟上时，重新发送同一目标会被误判为“已经在目标处”。 */
    AlignHost();

    const bool keep_reference = target_.ik.active && target_.ik.method == IkMethod::Numeric;

    if (!keep_reference)
    {
        for (int i = 0; i < kJointCount; ++i)
        {
            target_.ik.reference_deg[static_cast<size_t>(i)] = target_.joint[i].deg;
        }
        target_.ik.stall_cycles = 0;
    }
    target_.host.valid_mask = 0xFF;
    target_.host.closed_loop_mask = 0xFF;
    target_.host.align_pending = false;
    target_.align_pending = false;
    target_.ik.active = true;
    target_.ik.method = IkMethod::Numeric;
    target_.ik.pose = target;
    target_.ik.psi_mode = IkPsiMode::Manual;
    target_.ik.cost_weights = cost_weights;
    target_.ik.cost = 0.0f;
    target_.ik.branch = observer_.posture.branch;
    target_.ik.status = IkStatus::Planning;
    target_.ik.shoulder_singular = false;
    target_.ik.wrist_singular = false;
    target_.ik.converged = false;
    target_.ik.limit_mask = 0;
    target_.ik.step_limited = false;
    target_.ik.planning = true;
    target_.ik.plan_ready = false;
    ++target_.ik.plan_sequence;
    target_.ik.plan_cycles = 0;
    target_.ik.plan_failure = static_cast<uint8_t>(NumericPlanFailure::None);
    target_.ik.plan_time_us = 0.0;
    target_.ik.trajectory.clear();
    JointArray start_q{};
    for (int i = 0; i < kJointCount; ++i)
    {
        start_q[static_cast<size_t>(i)] = target_.joint[i].deg;
        target_.ik.plan_q_deg[static_cast<size_t>(i)] = start_q[static_cast<size_t>(i)];
    }
    target_.ik.plan_start_pose = ForwardKinematics(config_.dh, start_q);
    target_.ik.plan_goal_pose = target;
    const double dx = target(0, 3) - target_.ik.plan_start_pose(0, 3);
    const double dy = target(1, 3) - target_.ik.plan_start_pose(1, 3);
    const double dz = target(2, 3) - target_.ik.plan_start_pose(2, 3);
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double rotation = RotationDistanceRad(target_.ik.plan_start_pose, target);
    const double pos_steps = distance / config_.ik_numeric.plan_cartesian_step_m;
    const double rot_steps = rotation / config_.ik_numeric.plan_rotation_step_rad;
    target_.ik.plan_waypoint_count = std::max(
        1, static_cast<int>(std::ceil(std::max(pos_steps, rot_steps))));
    target_.ik.plan_waypoint_index = 0;
    target_.ik.trajectory.reserve(std::min<size_t>(target_.ik.plan_waypoint_count, 4096));
    target_.ik.trajectory_index = 0;
    return IkTargetResult::Ok;
}

void Arm::StepNumericIk()
{
    if (target_.ik.planning)
    {
        StepNumericPlan();
    }
    else if (target_.ik.plan_ready)
    {
        StepNumericTrajectory();
    }
}

/* 规划使用独立的历史构型 plan_q_deg，不修改 target_.joint，因此机械臂在计算阶段保持不动。
 * 每周期只执行有限数量迭代，避免完整规划阻塞实时控制。 */
void Arm::StepNumericPlan()
{
    const NumericIkParam &param = config_.ik_numeric;
    IkTarget &ik = target_.ik;
    JointArray limit_min{};
    JointArray limit_max{};

    const auto started = std::chrono::steady_clock::now();
    for (int i = 0; i < kJointCount; ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        limit_min[idx] = target_.joint[i].limit_min_deg;
        limit_max[idx] = target_.joint[i].limit_max_deg;
    }

    for (int iteration = 0; iteration < param.plan_iterations_per_cycle; ++iteration)
    {
        if (ik.plan_waypoint_index >= ik.plan_waypoint_count)
        {
            ik.planning = false;
            ik.plan_ready = true;
            ik.converged = true;
            ik.status = IkStatus::Ok;
            break;
        }
        const double path_fraction =
            static_cast<double>(ik.plan_waypoint_index + 1) /
            static_cast<double>(ik.plan_waypoint_count);
        const Mat4 waypoint = InterpolatePose(ik.plan_start_pose, ik.plan_goal_pose, path_fraction);
        const NumericIkStep step = SolveNumericIkStep(
            config_.dh, ik.plan_q_deg, waypoint, ik.reference_deg, ik.cost_weights,
            limit_min, limit_max, param);
        if (!step.ok)
        {
            ik.planning = false;
            ik.plan_ready = false;
            ik.trajectory.clear();
            ik.status = IkStatus::OutOfReach;
            ik.plan_failure = static_cast<uint8_t>(NumericPlanFailure::SolverFailed);
            break;
        }

        ik.pos_err_m = static_cast<float>(step.pos_err_m);
        ik.rot_err_deg = static_cast<float>(step.rot_err_rad * kRadToDeg);
        ik.limit_mask = step.limit_mask;
        ik.step_limited = step.step_limited;
        if (step.converged)
        {
            ++ik.plan_waypoint_index;
            ik.stall_cycles = 0;
            continue;
        }

        JointArray next{};
        float peak_deg = 0.0f;
        for (int i = 0; i < kJointCount; ++i)
        {
            const size_t idx = static_cast<size_t>(i);
            next[idx] = Clamp(ik.plan_q_deg[idx] + step.dq_deg[idx], limit_min[idx], limit_max[idx]);
            peak_deg = std::fmax(peak_deg, std::fabs(next[idx] - ik.plan_q_deg[idx]));
        }
        ik.plan_q_deg = next;
        ik.trajectory.push_back(next);
        ++ik.plan_cycles;

        constexpr float kPlanStallStepDeg = 1e-3f;
        constexpr uint16_t kPlanStallCycles = 250;
        if (peak_deg < kPlanStallStepDeg)
        {
            if (ik.stall_cycles < kPlanStallCycles)
            {
                ++ik.stall_cycles;
            }
        }
        else
        {
            ik.stall_cycles = 0;
        }
        if (ik.stall_cycles >= kPlanStallCycles)
        {
            ik.planning = false;
            ik.plan_ready = false;
            ik.trajectory.clear();
            ik.status = IkStatus::OutOfReach;
            ik.plan_failure = static_cast<uint8_t>(NumericPlanFailure::LimitOrStall);
            break;
        }
        if (ik.plan_cycles >= static_cast<uint32_t>(param.max_plan_cycles))
        {
            ik.planning = false;
            ik.plan_ready = false;
            ik.trajectory.clear();
            ik.status = IkStatus::OutOfReach;
            ik.plan_failure = static_cast<uint8_t>(NumericPlanFailure::MaxIterations);
            break;
        }
    }

    const auto finished = std::chrono::steady_clock::now();
    ik.plan_time_us += std::chrono::duration<double, std::micro>(finished - started).count();
}

/* 规划完成后按预计算轨迹逐周期推进。轨迹已经过限位和奇异检查，执行层不再采样求解；
 * 每周期只更新到下一个规划点，轨迹结束后持续保持最终关节目标。 */
void Arm::StepNumericTrajectory()
{
    IkTarget &ik = target_.ik;
    if (ik.trajectory_index >= ik.trajectory.size())
    {
        ik.converged = true;
        ik.status = IkStatus::Ok;
        return;
    }

    const JointArray &next = ik.trajectory[ik.trajectory_index];
    for (int i = 0; i < kJointCount; ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        target_.joint[i].deg = Clamp(next[idx], target_.joint[i].limit_min_deg,
                                     target_.joint[i].limit_max_deg);
    }
    ++ik.trajectory_index;
    ik.converged = ik.trajectory_index >= ik.trajectory.size();
    ik.status = IkStatus::Ok;
}

void Arm::StopHostControl()
{
    target_.host.closed_loop_mask = 0;
    for (int i = 0; i < kJointCount; ++i)
    {
        control_.output.joint_nm[i] = 0.0f;
        control_.joint_pid[i].Reset();
    }
    if (status_.state == State::HostIk)
    {
        target_.ik.active = false;
        target_.ik.planning = false;
        target_.ik.plan_ready = false;
        target_.ik.trajectory.clear();
        target_.ik.trajectory_index = 0;
        SetState(State::Host);
    }
}

void Arm::SetGripperTarget(float target_m)
{
    if (std::isfinite(target_m))
    {
        target_.gripper_m = Clamp(target_m, config_.gripper.open_m, config_.gripper.closed_m);
    }
}

/*--------------------------------------控制--------------------------------------*/
/* PID（ENABLE 全关节，HOST 只对闭环掩码里的关节）+ 重力前馈，再统一做总力矩限幅。
 * 方向折算在写执行器时做，执行器只认物理轴；非使能状态发零力矩 */
void Arm::ControlJoints()
{
    if (status_.state != State::Enable && status_.state != State::Host && status_.state != State::HostIk)
    {
        for (JointActuator &actuator : hardware_.arm)
        {
            actuator.SetEffort(0.0);
        }
        return;
    }

    for (int i = 0; i < kJointCount; ++i)
    {
        const float limit = config_.gain[i].total_output_max;
        const bool closed_loop = status_.state == State::Enable || status_.state == State::HostIk ||
                                 (target_.host.closed_loop_mask & (1U << i)) != 0U;
        float torque = 0.0f;

        if (closed_loop)
        {
            /* 误差直接相减，不做最短路径折叠，理由同实机 arm_jointctrl.c */
            torque = control_.joint_pid[i].Update(target_.joint[i].deg - observer_.joint.deg[i]);
        }
        else
        {
            control_.joint_pid[i].Reset();
        }
        torque += control_.gravity[i + 1];   // [0] 是腰部
        /* limit<=0 或结果非有限时输出 0 */
        const float output = (limit > 0.0f && std::isfinite(torque)) ? Clamp(torque, -limit, limit) : 0.0f;

        control_.output.joint_nm[i] = output;
        const float direction = config_.direction[i] < 0 ? -1.0f : 1.0f;
        hardware_.arm[i].SetEffort(direction * output);
    }
}

/* 腰部定在 target_.waist_deg，夹爪跟随目标。这两个不进上位机协议，使能后一直保持，
 * 主臂的 HOST 闭环掩码和掉线保护都不影响它们 */
void Arm::ControlWaistAndGripper(bool active)
{
    Output &out = control_.output;

    if (!active)
    {
        control_.waist_pid.Reset();
        control_.gripper_pid.Reset();
        out.waist_nm = 0.0f;
        out.gripper_n = 0.0f;
        hardware_.waist.SetEffort(0.0);
        for (JointActuator &finger : hardware_.finger)
        {
            finger.SetEffort(0.0);
        }
        return;
    }

    const float waist_limit = config_.waist_gain.total_output_max;
    const float waist_torque = control_.waist_pid.Update(target_.waist_deg - observer_.waist.deg) + control_.gravity[0];

    out.waist_nm = (waist_limit > 0.0f && std::isfinite(waist_torque))
                       ? Clamp(waist_torque, -waist_limit, waist_limit) : 0.0f;
    hardware_.waist.SetEffort(out.waist_nm);

    /* 误差 mm -> 每指力 N，两指受同样的力（+ 为夹紧），对称约束在 MJCF 里 */
    const float error_mm = (target_.gripper_m - observer_.gripper.m) * 1000.0f;
    out.gripper_n = control_.gripper_pid.Update(error_mm);
    for (JointActuator &finger : hardware_.finger)
    {
        finger.SetEffort(out.gripper_n);
    }
}

void Arm::Step()
{
    Observe();
    control_.gravity = gravity_.Compute(observer_.waist.deg, observer_.joint.deg);

    const bool active = status_.state == State::Enable || status_.state == State::Host ||
                        status_.state == State::HostIk;

    if (active)
    {
        status_.enabled = true;
    }
    UpdateTarget();
    ControlJoints();
    ControlWaistAndGripper(active);
}
