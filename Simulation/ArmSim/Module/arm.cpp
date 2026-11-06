#include "arm.h"

#include <cmath>
#include <stdexcept>

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

}  // namespace

const char *ToString(Arm::State state)
{
    static const char *const kNames[] = {"PROTECTED", "ENABLE", "FEEDBACK", "MASTER",
                                         "AUTO", "SERVANT", "HOST"};
    const auto index = static_cast<size_t>(state);

    return index < sizeof(kNames) / sizeof(kNames[0]) ? kNames[index] : "?";
}

/*--------------------------------------构造--------------------------------------*/
Arm::Arm(MujocoWorld &world, const Config &config)
    : config_(config),
      gravity_(config.gravity, config.dh),
      state_(config.init_state),
      host_align_pending_(config.init_state == State::Host)
{
    if (world.joint_count() < kJointCount)
    {
        throw std::invalid_argument("model has fewer joints than the arm needs");
    }
    actuator_.reserve(kJointCount);
    for (int i = 0; i < kJointCount; ++i)
    {
        actuator_.emplace_back(world, i);
        joint_[i].pid = Pid(config.gain[i]);
    }
}

/*--------------------------------------状态--------------------------------------*/
bool Arm::StateAllowed(State state) const
{
    switch (state)
    {
    case State::Protected:
    case State::Enable:
    case State::Host:
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
    if (state == state_)
    {
        return true;
    }

    if (state == State::Protected)
    {
        enabled_ = false;
    }
    state_ = state;
    ctrl_align_pending_ = true;
    host_align_pending_ = state == State::Host;
    host_closed_loop_mask_ = 0;
    host_target_valid_mask_ = 0;
    return true;
}

/*--------------------------------------观测--------------------------------------*/
/* 执行器读数 -> 关节量（按 direction 和零点折算）-> 正运动学 -> 末端位姿 */
void Arm::Observe()
{
    Observer &obs = observer_;

    ++cycle_count_;
    for (int i = 0; i < kJointCount; ++i)
    {
        const float direction = config_.direction[i] < 0 ? -1.0f : 1.0f;
        const float angle_rad = static_cast<float>(actuator_[i].angle_rad()) - config_.zero_point_rad[i];
        const float abs_rad = WrapPi(angle_rad);

        obs.joint_deg[i] = direction * angle_rad * kRadToDeg;
        obs.joint_abs_deg[i] = direction * abs_rad * kRadToDeg;
        obs.joint_vel_dps[i] = static_cast<float>(actuator_[i].velocity_rads()) * kRadToDeg * direction;
        obs.joint_torque_nm[i] = static_cast<float>(actuator_[i].torque_nm()) * direction;
        obs.encoder[i] = static_cast<uint16_t>(
            static_cast<int32_t>(std::lround(abs_rad / kTwoPi * kEncoderCpr)) &
            static_cast<int32_t>(kEncoderCpr - 1.0f));
    }
    obs.pose = PoseFromTransform(ForwardKinematics(config_.dh, obs.joint_deg));
}

/*--------------------------------------目标--------------------------------------*/
/* 限位映射到当前圈（差值不做环绕），目标对齐当前角，PID 清零 */
void Arm::AlignTarget()
{
    for (int i = 0; i < kJointCount; ++i)
    {
        JointCtrl &ctrl = joint_[i];
        const JointLimit &limit = config_.limit[i];
        const float multi_deg = observer_.joint_deg[i];
        const float abs_deg = observer_.joint_abs_deg[i];

        ctrl.limit_min_deg = multi_deg + (limit.min_deg - abs_deg);
        ctrl.limit_max_deg = multi_deg + (limit.max_deg - abs_deg);
        ctrl.target_deg = Clamp(multi_deg, ctrl.limit_min_deg, ctrl.limit_max_deg);
        ctrl.output = 0.0f;
        ctrl.pid.Reset();
    }
    ctrl_align_pending_ = false;
}

/* HOST 的对齐：在 AlignTarget 之上再清掉上位机的目标和闭环掩码 */
void Arm::AlignHost()
{
    AlignTarget();
    host_closed_loop_mask_ = 0;
    host_target_valid_mask_ = 0;
    host_align_pending_ = false;
}

void Arm::UpdateTarget()
{
    if (state_ == State::Host && host_align_pending_)
    {
        AlignHost();
    }
    else if (state_ == State::Enable && ctrl_align_pending_)
    {
        AlignTarget();
    }

    for (int i = 0; i < kJointCount; ++i)
    {
        JointCtrl &ctrl = joint_[i];

        switch (state_)
        {
        case State::Enable:   // 目标每周期跟随观测，靠重力前馈托住
            ctrl.target_deg = Clamp(observer_.joint_deg[i], ctrl.limit_min_deg, ctrl.limit_max_deg);
            break;
        case State::Host:     // 目标由 SetHostTarget 写入，这里只做限位
            ctrl.target_deg = Clamp(ctrl.target_deg, ctrl.limit_min_deg, ctrl.limit_max_deg);
            break;
        default:
            ctrl.target_deg = observer_.joint_deg[i];
            break;
        }
    }
}

Arm::HostTargetResult Arm::SetHostTarget(int joint, float target_abs_deg, bool closed_loop)
{
    if (joint < 0 || joint >= kJointCount)
    {
        return HostTargetResult::BadTarget;
    }
    if (state_ != State::Host)
    {
        return HostTargetResult::WrongState;
    }
    const JointLimit &limit = config_.limit[joint];
    if (!(target_abs_deg >= limit.min_deg && target_abs_deg <= limit.max_deg))
    {
        return HostTargetResult::BadTarget;
    }

    /* 第一次 HOST 控制前可能先收到目标：先锁存限位，再写目标 */
    if (host_align_pending_)
    {
        AlignHost();
    }

    const uint8_t bit = static_cast<uint8_t>(1U << joint);
    const bool was_closed_loop = (host_closed_loop_mask_ & bit) != 0U;

    joint_[joint].target_deg = observer_.joint_deg[joint] + (target_abs_deg - observer_.joint_abs_deg[joint]);
    host_target_valid_mask_ |= bit;
    if (closed_loop)
    {
        host_closed_loop_mask_ |= bit;
    }
    else
    {
        host_closed_loop_mask_ &= static_cast<uint8_t>(~bit);
    }
    if (closed_loop != was_closed_loop)
    {
        joint_[joint].pid.Reset();
    }
    return HostTargetResult::Ok;
}

void Arm::StopHostControl()
{
    host_closed_loop_mask_ = 0;
    for (JointCtrl &ctrl : joint_)
    {
        ctrl.output = 0.0f;
        ctrl.pid.Reset();
    }
}

/*--------------------------------------控制--------------------------------------*/
/* active=false 时发零力矩；方向折算在这里做，执行器只认物理轴 */
void Arm::SendTorque(bool active)
{
    for (int i = 0; i < kJointCount; ++i)
    {
        const float direction = config_.direction[i] < 0 ? -1.0f : 1.0f;

        actuator_[i].SetTorque(active ? direction * joint_[i].output : 0.0f);
    }
}

/* PID（ENABLE 全关节，HOST 只对闭环掩码里的关节）+ 重力前馈，再统一做总力矩限幅 */
void Arm::ControlJoints()
{
    if (state_ != State::Enable && state_ != State::Host)
    {
        SendTorque(false);
        return;
    }

    for (int i = 0; i < kJointCount; ++i)
    {
        JointCtrl &ctrl = joint_[i];
        const float limit = config_.gain[i].total_output_max;
        const bool closed_loop = state_ == State::Enable || (host_closed_loop_mask_ & (1U << i)) != 0U;
        float torque = 0.0f;

        if (closed_loop)
        {
            /* 误差直接相减，不做最短路径折叠，理由同实机 arm_jointctrl.c */
            torque = ctrl.pid.Update(ctrl.target_deg - observer_.joint_deg[i]);
        }
        else
        {
            ctrl.pid.Reset();
        }
        torque += gravity_cmd_[i];
        /* limit<=0 或结果非有限时输出 0 */
        ctrl.output = (limit > 0.0f && std::isfinite(torque)) ? Clamp(torque, -limit, limit) : 0.0f;
    }
    SendTorque(true);
}

void Arm::Step()
{
    Observe();
    gravity_cmd_ = gravity_.Compute(observer_.joint_deg);

    if (state_ == State::Enable || state_ == State::Host)
    {
        enabled_ = true;
    }
    UpdateTarget();
    ControlJoints();
}
