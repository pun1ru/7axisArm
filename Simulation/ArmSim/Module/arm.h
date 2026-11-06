#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "gravity.h"
#include "ik.h"
#include "ik_cost.h"
#include "joint_actuator.h"
#include "kinematics.h"
#include "mujoco_world.h"
#include "pid.h"

/* 机械臂：腰部 1 个关节（只负责直立）+ 主臂 7 个关节 + 夹爪（两指同步）。
 * 状态机 + 每关节 PID + 全链重力前馈。上位机协议只覆盖主臂 7 个关节，
 * 腰部和夹爪由 Arm 自己保持：腰部定在 waist_hold_deg，夹爪跟随 SetGripperTarget。
 * 主臂的 direction 和 zero_point 在这里处理：关节量 = direction * (物理量 - zero_point)。
 *
 * 每周期 Step()，数据只朝一个方向流：
 *
 *   Hardware(执行器) ──读──► Observer(观测) ──► Target(目标) ──► Control(控制) ──写──► Hardware
 *                              │ 关节量、末端位姿                 │ PID、重力前馈、输出力矩
 *                              │ 臂角 ψ、分支                     │
 *                              └─ 只由 Observe() 写               └─ Target 由 UpdateTarget() 和 SetXxxTarget() 写
 *                                                                    Control 只由 Control 阶段写
 *
 * 谁写谁：Observer 只有 Observe() 写；Target 只有 UpdateTarget()/Align*()/SetXxxTarget() 写；
 * Control 只有 ControlJoints()/ControlWaistAndGripper() 写。对外只读。 */
class Arm {
public:
    enum class Role : uint8_t { Master = 0, Servant };

    enum class State : uint8_t {
        Protected = 0,  // 保护：所有执行器出力恒为 0
        Enable,         // 使能：主臂全部关节闭环，目标跟随当前关节角
        Feedback,       // 主机：只读反馈
        Master,         // 主机：正常工作
        Auto,           // 从机：自主运行
        Servant,        // 从机：跟随主机
        Host,           // 上位机设定单关节目标，是否闭环由上位机决定
        HostIk          // 上位机设定末端位姿和臂角，本机解析逆解后闭环到关节目标
    };

    enum class HostTargetResult { Ok, WrongState, BadTarget };
    enum class IkTargetResult { Ok, WrongState, BadParam, BadBranch, OutOfReach, RefDegenerate, OutOfLimit };

    /*==================================== 配置（构造后不变） ====================================*/
    struct JointLimit {
        float min_deg;
        float max_deg;
    };

    /* 误差 deg -> 力矩 N*m。total_output_max 限制 PID + 重力前馈的最终总和，<=0 时该关节输出恒为 0 */
    struct JointGain : Pid::Gain {
        float total_output_max = 0.0f;
    };

    struct GripperConfig {
        std::array<std::string, 2> finger_joint;   // 两个滑动关节名
        float open_m = 0.0f;                       // 张开位置（装配位姿）
        float closed_m = 0.015f;                   // 夹紧位置（机械限位）
        Pid::Gain gain;                            // 误差 mm -> 每指力 N
        double finger_effort_limit = 50.0;         // 每指出力上限 N
    };

    struct Config {
        Role role = Role::Master;
        State init_state = State::Protected;

        // 腰部：转轴竖直，只起直立作用，定在 waist_hold_deg
        std::string waist_joint;
        float waist_hold_deg = 0.0f;
        JointGain waist_gain;
        Mat4 waist_mount = Mat4::Identity();       // 腰部转角为 0 时，腰部连杆坐标系相对世界的位姿

        // 主臂 7 个关节，DH 表对应这 7 个关节
        std::array<std::string, kJointCount> joint_name;
        double joint_effort_limit = 100.0;         // N*m
        DhTable dh{};
        std::array<float, kJointCount> zero_point_rad{};
        std::array<int8_t, kJointCount> direction{};
        std::array<JointLimit, kJointCount> limit{};
        std::array<JointGain, kJointCount> gain{};

        GripperConfig gripper;
        GravityModel::Param gravity;               // link[0] 是腰部连杆，link[1..7] 是主臂连杆
    };

    /*==================================== 观测层 ====================================
     * 全部由 Observe() 从执行器读数算出，下标 0..6 = 主臂第 1..7 个关节。 */
    struct JointObs {
        JointArray deg{};            // 多圈关节角
        JointArray abs_deg{};        // 单圈绝对角 (-180, 180]
        JointArray vel_dps{};
        JointArray torque_nm{};      // 上一周期写进执行器的力矩（折算过方向）
        std::array<uint16_t, kJointCount> encoder{};   // 单圈编码器值，按 16 bit 模拟
    };

    struct WaistObs {
        float deg = 0.0f;
        float torque_nm = 0.0f;
    };

    struct GripperObs {
        float m = 0.0f;              // 两指平均开合量：0 = 张开，closed_m = 夹紧
        float force_n = 0.0f;
    };

    /* 冗余度相关的构型量，只在 DH 表满足 S-R-S 时有效（见 kinematics.h） */
    struct Posture {
        ArmAngle arm_angle;          // 臂角 ψ：以 θ3 = 0 的臂平面为参考，定义见 ik.h
        Branch branch;               // 肩 λ2、肘 λ4、腕 λ6 各两个分支，共 8 个
    };

    struct Observer {
        WaistObs waist;
        JointObs joint;
        Pose pose;                   // 正运动学得到的末端位姿（相对腰部连杆坐标系）
        Posture posture;
        GripperObs gripper;
    };

    /*==================================== 目标层 ====================================
     * 控制层要追的东西，以及它们的约束。 */
    struct JointTarget {
        float deg = 0.0f;            // 多圈目标角，已限位
        float limit_min_deg = 0.0f;  // 限位映射到当前圈（差值不做环绕）
        float limit_max_deg = 0.0f;
    };

    /* 上位机协议的状态：哪些关节收到过目标、哪些关节由上位机要求闭环 */
    struct HostTarget {
        uint8_t closed_loop_mask = 0;   // bit i = 主臂第 i+1 个关节
        uint8_t valid_mask = 0;
        bool align_pending = false;     // 进入 HOST 后，等第一次 Step 或第一条目标时对齐
    };

    /* HOST_IK 的目标。target 是相对腰部连杆坐标系的目标位姿，psi_deg 是最终采用的臂角。 */
    struct IkTarget {
        bool active = false;
        Mat4 pose = Mat4::Identity();
        float psi_deg = 0.0f;
        IkPsiMode psi_mode = IkPsiMode::Manual;
        IkCostWeights cost_weights{};
        float cost = 0.0f;
        Branch branch;
        IkStatus status = IkStatus::Ok;
        bool shoulder_singular = false;
        bool wrist_singular = false;
    };

    struct Target {
        std::array<JointTarget, kJointCount> joint;
        float waist_deg = 0.0f;         // 腰部保持角
        float gripper_m = 0.0f;         // 夹爪目标开合量
        HostTarget host;
        IkTarget ik;
        bool align_pending = true;      // 目标需要对齐当前观测（状态切换后置位）
    };

    /*==================================== 控制层 ====================================
     * 目标 + 观测 -> 力矩。output 是最后一次写给执行器的值（方向折算之前）。 */
    struct Output {
        std::array<float, kJointCount> joint_nm{};
        float waist_nm = 0.0f;
        float gripper_n = 0.0f;         // 每指力，两指相同，+ 为夹紧
    };

    struct Control {
        std::array<Pid, kJointCount> joint_pid;
        Pid waist_pid;
        Pid gripper_pid;
        GravityModel::Dof gravity{};    // [0] 腰部，[1..7] 主臂
        Output output;

        explicit Control(const Config &config);
    };

    /*==================================== 运行状态 ====================================*/
    struct Status {
        State state = State::Protected;
        bool enabled = false;           // 进过 Enable/Host 之后为真，回到 Protected 清零
        uint32_t cycle_count = 0;       // Observe() 次数
    };

    /* world 必须比 Arm 活得久；关节名在 world 里找不到会抛 std::runtime_error */
    Arm(MujocoWorld &world, const Config &config);

    void Step();

    /* 状态由上层（脚本 / 上位机）修改，目标状态不属于本机角色时拒绝 */
    bool SetState(State state);
    /* HOST 单关节目标：默认 target_deg 为多圈角，必须落在进入 HOST 时锁存的多圈限位内。
     * multi_turn=false 时按旧协议解释为单圈绝对角，并映射到当前圈。 */
    HostTargetResult SetHostTarget(int joint, float target_deg, bool closed_loop,
                                   bool multi_turn = true);
    /* HOST_IK 目标：从 HOST 进入时校验请求分支与当前分支一致；成功后全部关节投入闭环。
     * target 是 ForwardKinematics 同一坐标系的末端位姿。Manual 使用 psi_deg，
     * Auto 忽略 psi_deg，按当前分支和 cost_weights 自动选择臂角。 */
    IkTargetResult SetIkTarget(const Mat4 &target, const Branch &branch, IkPsiMode psi_mode,
                               float psi_deg, const IkCostWeights &cost_weights);
    /* 主臂全部关节退出闭环，目标保留，只剩重力前馈；腰部和夹爪继续保持 */
    void StopHostControl();
    /* 夹爪目标开合量 m，限制在 [open_m, closed_m] */
    void SetGripperTarget(float target_m);

    const Config &config() const { return config_; }
    const Status &status() const { return status_; }
    const Observer &observer() const { return observer_; }
    const Target &target() const { return target_; }
    const Control &control() const { return control_; }

    State state() const { return status_.state; }
    bool enabled() const { return status_.enabled; }
    uint32_t cycle_count() const { return status_.cycle_count; }
    bool aligning() const { return target_.align_pending || target_.host.align_pending; }
    /* DH 表是 S-R-S 构型，臂角和分支才有意义 */
    bool srs() const { return srs_valid_; }
    const SrsParam &srs_param() const { return srs_; }

private:
    /* 执行器：腰部 1 个 + 主臂 7 个 + 夹爪两指。JointActuator 没有默认构造，所以主臂和夹爪用 vector 就地构造 */
    struct Hardware {
        JointActuator waist;
        std::vector<JointActuator> arm;      // 主臂 7 个
        std::vector<JointActuator> finger;   // 夹爪两指

        Hardware(MujocoWorld &world, const Config &config);
    };

    bool StateAllowed(State state) const;

    // 观测
    void Observe();
    // 目标
    void AlignTarget();
    void AlignHost();
    void UpdateTarget();
    // 控制
    void ControlJoints();
    void ControlWaistAndGripper(bool active);

    Config config_;
    Hardware hardware_;
    GravityModel gravity_;
    SrsParam srs_;
    bool srs_valid_;
    Status status_;
    Observer observer_;
    Target target_;
    Control control_;
};

const char *ToString(Arm::State state);
