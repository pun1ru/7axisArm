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

    /* 连杆（body）的质量和质心。in_parent=false：质心在 body 自身坐标系；
     * in_parent=true：折算到父 body 坐标系（q=0 时），用于把夹爪并入 link8 */
    Inertial BodyInertial(const std::string &body, bool in_parent) const;

    /* 给 Viewer 用的原始指针，Viewer 之外不要碰 */
    mjModel *model() { return model_; }
    mjData *data() { return data_; }

    double time() const;
    double timestep() const;
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
    std::vector<JointAddr> joint_;
    int warning_count_ = 0;
    unsigned step_count_ = 0;
};
