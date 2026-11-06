#include "mujoco_world.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace {

/* MuJoCo 日志回调：把引擎 warning 打到 stderr */
void LogWarning(const char *msg)
{
    std::fprintf(stderr, "[mujoco] %s\n", msg);
}

}  // namespace

MujocoWorld::MujocoWorld(const Config &config)
    : torque_limit_nm_(config.torque_limit_nm > 0.0 ? config.torque_limit_nm : 100.0)
{
    char err[1024] = {};

    model_ = mj_loadXML(config.xml_path.c_str(), nullptr, err, sizeof(err));
    if (model_ == nullptr)
    {
        throw std::runtime_error("load " + config.xml_path + " failed:\n" + err);
    }

    /* armature / damping / frictionloss 是电机侧的模型，不在 URDF 里。
     * 远端关节惯量很小（J6 约 1.5e-5 kg*m^2），不加时力矩控制会抖、会慢漂。 */
    for (int i = 0; i < model_->nv; ++i)
    {
        if (config.joint_armature > 0.0)
        {
            model_->dof_armature[i] = config.joint_armature;
        }
        if (config.joint_damping > 0.0)
        {
            model_->dof_damping[i] = config.joint_damping;
        }
        if (config.joint_frictionloss > 0.0)
        {
            model_->dof_frictionloss[i] = config.joint_frictionloss;
        }
    }

    data_ = mj_makeData(model_);
    if (data_ == nullptr)
    {
        mj_deleteModel(model_);
        throw std::runtime_error("mj_makeData failed");
    }

    /* 约定关节名 joint1..jointN，找不到就认为链结束 */
    for (int i = 0; i < kMaxJoints; ++i)
    {
        const std::string name = "joint" + std::to_string(i + 1);
        const int jid = mj_name2id(model_, mjOBJ_JOINT, name.c_str());

        if (jid < 0)
        {
            break;
        }
        qpos_adr_[i] = model_->jnt_qposadr[jid];
        dof_adr_[i] = model_->jnt_dofadr[jid];
        joint_count_ = i + 1;
    }
    if (joint_count_ == 0)
    {
        mj_deleteData(data_);
        mj_deleteModel(model_);
        throw std::runtime_error(config.xml_path + " has no joint1..jointN");
    }

    tcp_site_id_ = mj_name2id(model_, mjOBJ_SITE, "tcp");

    mju_user_warning = LogWarning;
    mj_forward(model_, data_);
}

MujocoWorld::~MujocoWorld()
{
    mj_deleteData(data_);
    mj_deleteModel(model_);
}

void MujocoWorld::Step()
{
    mj_step(model_, data_);
    ++step_count_;
    warning_count_ += data_->warning[0].number;
    data_->warning[0].number = 0;
}

double MujocoWorld::JointAngleRad(int joint) const
{
    return JointValid(joint) ? data_->qpos[qpos_adr_[joint]] : 0.0;
}

double MujocoWorld::JointVelocityRad(int joint) const
{
    return JointValid(joint) ? data_->qvel[dof_adr_[joint]] : 0.0;
}

double MujocoWorld::SetJointTorque(int joint, double torque_nm)
{
    if (!JointValid(joint))
    {
        return 0.0;
    }
    if (std::isnan(torque_nm))
    {
        torque_nm = 0.0;
    }
    torque_nm = std::clamp(torque_nm, -torque_limit_nm_, torque_limit_nm_);
    data_->qfrc_applied[dof_adr_[joint]] = torque_nm;
    return torque_nm;
}

double MujocoWorld::time() const
{
    return data_->time;
}

double MujocoWorld::timestep() const
{
    return model_->opt.timestep;
}

std::optional<std::array<double, 3>> MujocoWorld::TcpPosition() const
{
    if (tcp_site_id_ < 0)
    {
        return std::nullopt;
    }
    const mjtNum *p = data_->site_xpos + 3 * tcp_site_id_;
    return std::array<double, 3>{p[0], p[1], p[2]};
}
