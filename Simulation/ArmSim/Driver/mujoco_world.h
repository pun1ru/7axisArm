#pragma once

#include <array>
#include <optional>
#include <string>

/* MuJoCo 的模型/数据只以指针形式出现在这里，上层不需要 include mujoco.h */
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

/* 物理世界：包一层 MuJoCo，对上层提供“关节角 / 关节速度 / 关节力矩”口径的读写。
 * 力矩通过 qfrc_applied 注入，不依赖 XML 里的 actuator，nu=0 的模型也能直接跑。
 * 按名字找 joint1..jointN 和 "tcp" site。 */
class MujocoWorld {
public:
    struct Config {
        std::string xml_path;
        // 以下三项是电机侧的模型，不在 URDF 里；0 = 沿用 MJCF
        double joint_armature = 0.0;      // 关节反射转子惯量 [kg*m^2]
        double joint_damping = 0.0;       // 粘性阻尼 [N*m*s/rad]
        double joint_frictionloss = 0.0;  // 库仑摩擦 [N*m]
        double torque_limit_nm = 100.0;   // 单关节力矩安全限幅
    };

    /* 加载失败抛 std::runtime_error */
    explicit MujocoWorld(const Config &config);
    ~MujocoWorld();
    MujocoWorld(const MujocoWorld &) = delete;
    MujocoWorld &operator=(const MujocoWorld &) = delete;

    void Step();

    double JointAngleRad(int joint) const;
    double JointVelocityRad(int joint) const;
    /* 写进 qfrc_applied，内部限幅，NaN 当 0。返回实际写入的值 */
    double SetJointTorque(int joint, double torque_nm);

    double time() const;
    double timestep() const;
    int joint_count() const { return joint_count_; }
    int warning_count() const { return warning_count_; }
    unsigned step_count() const { return step_count_; }
    std::optional<std::array<double, 3>> TcpPosition() const;

private:
    static constexpr int kMaxJoints = 16;

    bool JointValid(int joint) const { return joint >= 0 && joint < joint_count_; }

    mjModel *model_ = nullptr;
    mjData *data_ = nullptr;
    int joint_count_ = 0;
    std::array<int, kMaxJoints> qpos_adr_{};   // jointN -> qpos 下标
    std::array<int, kMaxJoints> dof_adr_{};    // jointN -> qvel / qfrc 下标
    double torque_limit_nm_;
    int tcp_site_id_ = -1;
    int warning_count_ = 0;
    unsigned step_count_ = 0;
};
