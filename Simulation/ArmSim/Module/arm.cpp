#include "arm.h"

#include <cmath>
#include <limits>

namespace {

constexpr float kEncoderCpr = 65536.0f;

float Clamp(float value, float min_limit, float max_limit)
{
    return value > max_limit ? max_limit : (value < min_limit ? min_limit : value);
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
    }
    return true;
}

/*--------------------------------------观测--------------------------------------*/
/* 执行器读数 -> 关节量（按 direction 和零点折算）-> 正运动学 -> 末端位姿、臂角、分支 */
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

    obs.pose = PoseFromTransform(frames.back());
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
                                     float psi_deg, const IkCostWeights &cost_weights)
{
    if (status_.state != State::Host && status_.state != State::HostIk)
    {
        return IkTargetResult::WrongState;
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
            srs_, target, branch, observer_, config_, previous_target_deg, cost_weights,
            preferred_psi_deg, psi_deg, solved, next_target, cost);
        if (auto_result != IkTargetResult::Ok)
        {
            return auto_result;
        }
    }
    else
    {
        solved = SolveIk(srs_, target, branch, psi_deg);

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
    target_.ik.pose = target;
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
