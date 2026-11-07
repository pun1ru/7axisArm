#include "mujoco_world.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {

/* MuJoCo 日志回调：把引擎 warning 打到 stderr */
void LogWarning(const char *msg)
{
    std::fprintf(stderr, "[mujoco] %s\n", msg);
}

struct SpecGuard {
    mjSpec *spec = nullptr;

    ~SpecGuard()
    {
        if (spec != nullptr)
        {
            mj_deleteSpec(spec);
        }
    }
};

mjModel *LoadAttached(const MujocoWorld::Config &config)
{
    char error[1024] = {};
    /* 析构逆序：附件规格必须先于 arena 释放，否则 arena 已释放的 body
     * 仍会被附件规格的析构路径访问。 */
    SpecGuard arena;
    std::vector<SpecGuard> attachments;
    attachments.resize(config.exchange_xml_path.empty() ? 2 : 3);

    arena.spec = mj_parseXML(config.arena_path.c_str(), nullptr, error, sizeof(error));
    if (arena.spec == nullptr)
    {
        throw std::runtime_error("load arena " + config.arena_path + " failed:\n" + error);
    }
    /* mjs_attach 默认浅拷贝，源 spec 释放后 arena 会留下悬空指针。
     * 对目标启用深拷贝，物理/显示两套模型都能安全销毁。 */
    if (mjs_setDeepCopy(arena.spec, 1) != 0)
    {
        throw std::runtime_error("failed to enable deep copy for " + config.arena_path);
    }

    attachments[0].spec = mj_parseXML(config.xml_path.c_str(), nullptr, error, sizeof(error));
    if (attachments[0].spec == nullptr)
    {
        throw std::runtime_error("load robot " + config.xml_path + " failed:\n" + error);
    }

    if (config.remove_attachment_floor)
    {
        mjsElement *floor = mjs_findElement(attachments[0].spec, mjOBJ_GEOM, "floor");
        if (floor != nullptr && mjs_delete(attachments[0].spec, floor) != 0)
        {
            throw std::runtime_error("failed to remove attachment floor from " + config.xml_path);
        }
    }

    mjsBody *worldbody = mjs_findBody(arena.spec, "world");
    if (worldbody == nullptr)
    {
        throw std::runtime_error("arena model has no world body");
    }

    auto attach = [&](mjSpec *attachment, const std::string &path, const std::string &prefix,
                      const std::array<double, 3> &position,
                      const std::array<double, 4> &quaternion) {
        if (mjs_setDeepCopy(attachment, 1) != 0)
        {
            throw std::runtime_error("failed to enable deep copy for " + path);
        }
        mjsFrame *frame = mjs_addFrame(worldbody, nullptr);
        if (frame == nullptr)
        {
            throw std::runtime_error("failed to create attachment frame");
        }
        for (int i = 0; i < 3; ++i)
        {
            frame->pos[i] = position[i];
        }
        for (int i = 0; i < 4; ++i)
        {
            frame->quat[i] = quaternion[i];
        }
        if (mjs_attach(frame->element, attachment->element, prefix.c_str(), "") == nullptr)
        {
            throw std::runtime_error("failed to attach " + path + " to " + config.arena_path);
        }
    };

    attach(attachments[0].spec, config.xml_path, config.model_prefix,
           config.model_position, config.model_quaternion);

    if (!config.exchange_xml_path.empty())
    {
        attachments[1].spec = mj_parseXML(config.exchange_xml_path.c_str(), nullptr, error,
                                          sizeof(error));
        if (attachments[1].spec == nullptr)
        {
            throw std::runtime_error("load exchange station " + config.exchange_xml_path +
                                     " failed:\n" + error);
        }
        attach(attachments[1].spec, config.exchange_xml_path, config.exchange_prefix,
               config.exchange_position, config.exchange_quaternion);
    }

    mjModel *model = mj_compile(arena.spec, nullptr);
    if (model == nullptr)
    {
        const char *compile_error = mjs_getError(arena.spec);
        throw std::runtime_error(std::string("compile attached model failed:\n") +
                                 (compile_error == nullptr ? "unknown MuJoCo error" : compile_error));
    }
    if (config.remove_attachment_floor)
    {
        const std::string floor_name = config.model_prefix + "floor";
        if (mj_name2id(model, mjOBJ_GEOM, floor_name.c_str()) >= 0)
        {
            mj_deleteModel(model);
            throw std::runtime_error("attachment floor was not removed: " + floor_name);
        }
    }
    return model;
}

}  // namespace

MujocoWorld::MujocoWorld(const Config &config)
{
    char err[1024] = {};

    if (config.arena_path.empty())
    {
        model_ = mj_loadXML(config.xml_path.c_str(), nullptr, err, sizeof(err));
        model_prefix_.clear();
    }
    else
    {
        model_ = LoadAttached(config);
        model_prefix_ = config.model_prefix;
    }
    if (model_ == nullptr)
    {
        throw std::runtime_error("load " + config.xml_path + " failed:\n" + err);
    }

    /* armature / damping / frictionloss 是机械臂电机侧的模型，不在 URDF 里。
     * 只施加到机械臂附件；兑换站已有自己的执行器和机构阻尼，不能被这些
     * 参数二次覆盖，否则位置环会明显振荡。 */
    for (int i = 0; i < model_->njnt; ++i)
    {
        if (model_->jnt_type[i] == mjJNT_FREE || model_->jnt_type[i] == mjJNT_BALL)
        {
            continue;
        }
        const char *name = mj_id2name(model_, mjOBJ_JOINT, i);
        if (!config.model_prefix.empty() &&
            (name == nullptr || std::strncmp(name, config.model_prefix.c_str(),
                                             config.model_prefix.size()) != 0))
        {
            continue;
        }
        const int dof = model_->jnt_dofadr[i];
        if (config.joint_armature > 0.0)
        {
            model_->dof_armature[dof] = config.joint_armature;
        }
        if (config.joint_damping > 0.0)
        {
            model_->dof_damping[dof] = config.joint_damping;
        }
        if (config.joint_frictionloss > 0.0)
        {
            model_->dof_frictionloss[dof] = config.joint_frictionloss;
        }
    }

    data_ = mj_makeData(model_);
    if (data_ == nullptr)
    {
        mj_deleteModel(model_);
        throw std::runtime_error("mj_makeData failed");
    }

    if (!config.arena_path.empty() && !config.view_arena_path.empty())
    {
        try
        {
            Config view_config = config;
            view_config.arena_path = config.view_arena_path;
            view_model_ = LoadAttached(view_config);
            if (view_model_->nq != model_->nq || view_model_->nv != model_->nv)
            {
                mj_deleteModel(view_model_);
                view_model_ = nullptr;
                throw std::runtime_error("view model joint count differs from physics model");
            }
            view_data_ = mj_makeData(view_model_);
            if (view_data_ == nullptr)
            {
                mj_deleteModel(view_model_);
                view_model_ = nullptr;
                throw std::runtime_error("view mj_makeData failed");
            }
            std::printf("[mujoco] geometry: physics=%lld geoms/%lld meshes, "
                        "viewer=%lld geoms/%lld meshes\n",
                        static_cast<long long>(model_->ngeom),
                        static_cast<long long>(model_->nmesh),
                        static_cast<long long>(view_model_->ngeom),
                        static_cast<long long>(view_model_->nmesh));
        }
        catch (const std::exception &e)
        {
            std::fprintf(stderr, "[mujoco] 轻量显示模型加载失败，回退到完整模型: %s\n", e.what());
            view_model_ = model_;
            view_data_ = data_;
        }
    }
    else
    {
        view_model_ = model_;
        view_data_ = data_;
    }

    mju_user_warning = LogWarning;
    mj_forward(model_, data_);
    SyncViewerState();
}

MujocoWorld::~MujocoWorld()
{
    if (view_data_ != nullptr && view_data_ != data_)
    {
        mj_deleteData(view_data_);
    }
    if (view_model_ != nullptr && view_model_ != model_)
    {
        mj_deleteModel(view_model_);
    }
    mj_deleteData(data_);
    mj_deleteModel(model_);
}

void MujocoWorld::Step()
{
    /* 释放的子树：MuJoCo 内置 gravcomp 在这个模型上没起作用（qfrc_gravcomp 恒为 0），
     * 所以每步在各 body 质心处施加等大反向的重力，用 qfrc_applied 注入。 */
    if (!free_bodies_.empty())
    {
        for (int d : free_dofs_)
        {
            data_->qfrc_applied[d] = 0.0;
        }
        for (int b : free_bodies_)
        {
            const mjtNum f[3] = {-model_->body_mass[b] * model_->opt.gravity[0],
                                 -model_->body_mass[b] * model_->opt.gravity[1],
                                 -model_->body_mass[b] * model_->opt.gravity[2]};
            const mjtNum torque[3] = {0, 0, 0};
            mj_applyFT(model_, data_, f, torque, data_->xipos + 3 * b, b, data_->qfrc_applied);
        }
    }
    mj_step(model_, data_);
    SyncViewerState();
    ++step_count_;
    warning_count_ += data_->warning[0].number;
    data_->warning[0].number = 0;
}

void MujocoWorld::SyncViewerState()
{
    if (view_model_ == model_ || view_data_ == data_)
    {
        return;
    }
    std::copy(data_->qpos, data_->qpos + model_->nq, view_data_->qpos);
    std::copy(data_->qvel, data_->qvel + model_->nv, view_data_->qvel);
    std::copy(data_->ctrl, data_->ctrl + model_->nu, view_data_->ctrl);
    std::copy(data_->qfrc_applied, data_->qfrc_applied + model_->nv, view_data_->qfrc_applied);
    view_data_->time = data_->time;
    mj_forward(view_model_, view_data_);
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

int MujocoWorld::BindActuator(const std::string &name)
{
    const int id = mj_name2id(model_, mjOBJ_ACTUATOR, name.c_str());

    if (id < 0)
    {
        throw std::runtime_error("model has no actuator \"" + name + "\"");
    }
    actuator_.push_back(id);
    return static_cast<int>(actuator_.size()) - 1;
}

void MujocoWorld::SetActuatorControl(int handle, double value)
{
    const int id = actuator_.at(handle);
    const mjtNum lo = model_->actuator_ctrlrange[2 * id];
    const mjtNum hi = model_->actuator_ctrlrange[2 * id + 1];
    const double safe = std::isnan(value) ? 0.0 : value;
    data_->ctrl[id] = std::clamp(safe, lo, hi);
}

static_assert(mjNGAIN == 10 && mjNBIAS == 10, "actuator_saved_ 维度要和 MuJoCo 一致");

void MujocoWorld::SetActuatorEnabled(int handle, bool enabled)
{
    const int id = actuator_.at(handle);
    if (actuator_saved_.empty())
    {
        actuator_saved_.resize(actuator_.size());
        actuator_disabled_.assign(actuator_.size(), false);
    }
    auto &saved = actuator_saved_.at(handle);
    mjtNum *gain = model_->actuator_gainprm + id * mjNGAIN;
    mjtNum *bias = model_->actuator_biasprm + id * mjNBIAS;
    if (!enabled && !actuator_disabled_[handle])
    {
        std::copy(gain, gain + mjNGAIN, saved.begin());
        std::copy(bias, bias + mjNBIAS, saved.begin() + mjNGAIN);
        std::fill(gain, gain + mjNGAIN, 0.0);
        std::fill(bias, bias + mjNBIAS, 0.0);
        actuator_disabled_[handle] = true;
    }
    else if (enabled && actuator_disabled_[handle])
    {
        std::copy(saved.begin(), saved.begin() + mjNGAIN, gain);
        std::copy(saved.begin() + mjNGAIN, saved.end(), bias);
        actuator_disabled_[handle] = false;
    }
}

void MujocoWorld::SetSubtreeFree(const std::string &root_body, bool free)
{
    const int root = mj_name2id(model_, mjOBJ_BODY, root_body.c_str());
    if (root < 0)
    {
        throw std::runtime_error("model has no body \"" + root_body + "\"");
    }
    if (saved_frictionloss_.empty())
    {
        saved_frictionloss_.assign(model_->dof_frictionloss, model_->dof_frictionloss + model_->nv);
        saved_damping_.assign(model_->dof_damping, model_->dof_damping + model_->nv);
    }
    constexpr double kFreeDamping = 0.5;   // 手松开后不甩动，又不妨碍拖拽
    for (int d : free_dofs_)
    {
        data_->qfrc_applied[d] = 0.0;   // 清掉上一步残留的补偿力，否则退出释放后会一直推关节
    }
    free_bodies_.clear();
    free_dofs_.clear();
    for (int b = 1; b < model_->nbody; ++b)
    {
        bool in_subtree = false;
        for (int k = b; k > 0; k = model_->body_parentid[k])
        {
            in_subtree |= (k == root);
        }
        if (!in_subtree)
        {
            continue;
        }
        if (free && model_->body_mass[b] > 0.0)
        {
            free_bodies_.push_back(b);
        }
        for (int d = model_->body_dofadr[b]; d < model_->body_dofadr[b] + model_->body_dofnum[b]; ++d)
        {
            if (free)
            {
                free_dofs_.push_back(d);
            }
            model_->dof_frictionloss[d] = free ? 0.0 : saved_frictionloss_[d];
            model_->dof_damping[d] = free ? kFreeDamping : saved_damping_[d];
        }
    }
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
