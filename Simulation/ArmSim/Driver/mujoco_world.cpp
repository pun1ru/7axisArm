#include "mujoco_world.h"

#include <mujoco/mujoco.h>

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
{
    char err[1024] = {};

    model_ = mj_loadXML(config.xml_path.c_str(), nullptr, err, sizeof(err));
    if (model_ == nullptr)
    {
        throw std::runtime_error("load " + config.xml_path + " failed:\n" + err);
    }

    /* armature / damping / frictionloss 是电机侧的模型，不在 URDF 里。
     * 远端关节惯量很小，不加时力矩控制会抖、会慢漂。 */
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

int MujocoWorld::BindJoint(const std::string &name)
{
    const int jid = mj_name2id(model_, mjOBJ_JOINT, name.c_str());

    if (jid < 0)
    {
        throw std::runtime_error("model has no joint \"" + name + "\"");
    }
    joint_.push_back({model_->jnt_qposadr[jid], model_->jnt_dofadr[jid]});
    return static_cast<int>(joint_.size()) - 1;
}

double MujocoWorld::JointPosition(int handle) const
{
    return data_->qpos[joint_.at(handle).qpos];
}

double MujocoWorld::JointVelocity(int handle) const
{
    return data_->qvel[joint_.at(handle).dof];
}

void MujocoWorld::SetJointForce(int handle, double force)
{
    data_->qfrc_applied[joint_.at(handle).dof] = std::isnan(force) ? 0.0 : force;
}

MujocoWorld::Inertial MujocoWorld::BodyInertial(const std::string &body, bool in_parent) const
{
    const int id = mj_name2id(model_, mjOBJ_BODY, body.c_str());
    Inertial out;

    if (id < 0)
    {
        throw std::runtime_error("model has no body \"" + body + "\"");
    }
    out.mass_kg = model_->body_mass[id];

    mjtNum com[3] = {model_->body_ipos[3 * id], model_->body_ipos[3 * id + 1],
                     model_->body_ipos[3 * id + 2]};
    if (in_parent)
    {
        /* 父坐标系下 = body_pos + R(body_quat) * ipos（q=0 时的参考位姿） */
        mjtNum rotated[3];
        mju_rotVecQuat(rotated, com, model_->body_quat + 4 * id);
        for (int i = 0; i < 3; ++i)
        {
            com[i] = model_->body_pos[3 * id + i] + rotated[i];
        }
    }
    out.com_m = {com[0], com[1], com[2]};
    return out;
}

double MujocoWorld::time() const
{
    return data_->time;
}

double MujocoWorld::timestep() const
{
    return model_->opt.timestep;
}

std::optional<std::array<double, 3>> MujocoWorld::SitePosition(const std::string &site) const
{
    const int id = mj_name2id(model_, mjOBJ_SITE, site.c_str());

    if (id < 0)
    {
        return std::nullopt;
    }
    const mjtNum *p = data_->site_xpos + 3 * id;
    return std::array<double, 3>{p[0], p[1], p[2]};
}
