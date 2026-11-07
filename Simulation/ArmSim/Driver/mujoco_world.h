#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

/* MuJoCo 的模型/数据只以指针形式出现在这里，上层不需要 include mujoco.h */
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

/* 物理世界：包一层 MuJoCo，对上层提供“按名字绑定关节 -> 读位置/速度、写力”的口径。
 * 力通过 qfrc_applied 注入，不依赖 XML 里的 actuator。
 * 转动关节单位 rad、N*m；滑动关节单位 m、N。 */
class MujocoWorld {
public:
    struct Config {
        std::string xml_path;
        /* 非空时把 xml_path 作为附件合并到该场地 MJCF。场地拥有全局 option，
         * 附件的 option 不会覆盖场地；模型位置和姿态使用世界系，单位 m。 */
        std::string arena_path;
        /* 非空时使用轻量视觉场地给 Viewer，物理仍使用 arena_path。
         * 两者必须具有相同的机械臂关节结构。 */
        std::string view_arena_path;
        std::string model_prefix;
        std::array<double, 3> model_position{};
        std::array<double, 4> model_quaternion{{1.0, 0.0, 0.0, 0.0}};
        /* 可选兑换站附件。位置描述装配坐标系原点 O（底座正面与地面交线
         * 中点），单位 m；exchange_station.xml 的 CAD 底面比 O 低 78 mm。 */
        std::string exchange_xml_path;
        std::array<double, 3> exchange_position{};
        std::array<double, 4> exchange_quaternion{{1.0, 0.0, 0.0, 0.0}};
        std::string exchange_prefix = "exchange/";
        /* 合并场地时移除附件自带的 worldbody 地面，避免无限大 plane 覆盖场地地形。 */
        bool remove_attachment_floor = true;
        // 以下三项是电机侧的模型，不在 URDF 里；0 = 沿用 MJCF
        double joint_armature = 0.0;      // 关节反射转子惯量 [kg*m^2 或 kg]
        double joint_damping = 0.0;       // 粘性阻尼
        double joint_frictionloss = 0.0;  // 库仑摩擦
    };

    struct Inertial {
        double mass_kg = 0.0;
        std::array<double, 3> com_m{};    // 质心
    };

    /* 加载失败抛 std::runtime_error */
    explicit MujocoWorld(const Config &config);
    ~MujocoWorld();
    MujocoWorld(const MujocoWorld &) = delete;
    MujocoWorld &operator=(const MujocoWorld &) = delete;

    void Step();

    /* 按 MJCF 里的关节名绑定，返回句柄；找不到抛 std::runtime_error */
    int BindJoint(const std::string &name);
    double JointPosition(int handle) const;
    double JointVelocity(int handle) const;
    /* 写进 qfrc_applied；NaN 当 0。限幅由调用方（JointActuator）负责 */
    void SetJointForce(int handle, double force);
    /* 位置执行器（MJCF actuator）的控制量；主要用于兑换站机构。 */
    int BindActuator(const std::string &name);
    void SetActuatorControl(int handle, double value);
    /* false = 撤掉驱动：位置执行器的增益和偏置清零，关节不再被拉回目标。
     * 原参数在第一次撤销时保存，true 时恢复。 */
    void SetActuatorEnabled(int handle, bool enabled);
    /* 让 root_body 子树可以被手拖动：每步手动抵消重力 + 摩擦清零（加少量阻尼防抖）。
     * false 恢复模型原值。只改物理模型，查看器的拖拽力作用在同一份 mjData 上。 */
    void SetSubtreeFree(const std::string &root_body, bool free);

    /* 连杆（body）的质量和质心。in_parent=false：质心在 body 自身坐标系；
     * in_parent=true：折算到父 body 坐标系（q=0 时），用于把夹爪并入 link8 */
    Inertial BodyInertial(const std::string &body, bool in_parent) const;

    /* 给 Viewer 用的原始指针，Viewer 之外不要碰 */
    mjModel *model() { return model_; }
    mjData *data() { return data_; }
    mjModel *view_model() { return view_model_; }
    mjData *view_data() { return view_data_; }

    double time() const;
    double timestep() const;
    /* 场地附件使用的名称前缀；纯机械臂模式为空。 */
    const std::string &model_prefix() const { return model_prefix_; }
    int warning_count() const { return warning_count_; }
    unsigned step_count() const { return step_count_; }
    /* 某个 site 在世界系的位置；没有该 site 返回 nullopt。
     * mj_step 在积分前才做前向计算，所以这里是最近一次 Step 开始时的位置 */
    std::optional<std::array<double, 3>> SitePosition(const std::string &site) const;

private:
    struct JointAddr {
        int qpos;
        int dof;
    };

    mjModel *model_ = nullptr;
    mjData *data_ = nullptr;
    mjModel *view_model_ = nullptr;
    mjData *view_data_ = nullptr;
    std::string model_prefix_;
    std::vector<JointAddr> joint_;
    std::vector<int> actuator_;
    /* gainprm + biasprm，各 10 项（= mjNGAIN / mjNBIAS，cpp 里 static_assert 核对） */
    std::vector<std::array<double, 20>> actuator_saved_;
    std::vector<bool> actuator_disabled_;
    std::vector<double> saved_frictionloss_, saved_damping_;
    std::vector<int> free_bodies_, free_dofs_;   // 释放的子树：手动重力补偿用
    int warning_count_ = 0;
    unsigned step_count_ = 0;

    void SyncViewerState();
};
