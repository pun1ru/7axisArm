#pragma once

#include <cstdint>

#include "ik_cost.h"
#include "kinematics.h"

/* 七轴臂的数值逆解：每个控制周期在当前关节角处线性化，解一步 Δq。
 *
 * 记 q 为 7 个关节角（rad），e 为末端 6 维位姿误差，J 为 6×7 几何 Jacobian，W 为关节权重（对角），
 * q_ref 为参考构型。每步最小化
 *
 *     ‖J·Δq − e‖²  +  λ²·Δqᵀ W Δq                       （任务误差 + 加权阻尼）
 *
 * 闭式解（对偶形式，只需解 6×6 的对称正定方程）：
 *
 *     J̄  = W⁻¹ Jᵀ (J W⁻¹ Jᵀ + λ² I₆)⁻¹                  （加权阻尼伪逆）
 *     Δq = J̄ e  +  c·n,   c = nᵀW z / nᵀW n,   z = k·(q_ref − q)     （零空间项）
 *
 * 零空间项 dq_null = z − W⁻¹Jᵀ(J W⁻¹ Jᵀ)⁻¹ J z，是 z 在 J 零空间上的 W 加权精确投影（用无阻尼伪逆，J·dq_null = 0）。
 * 它在不影响末端的方向（臂角自运动）上把关节拉向参考构型，k 是每周期的拉回比例；有关节被限位固定后没有冗余度，
 * 该项为 0；J W⁻¹ Jᵀ 秩亏（Cholesky 失败）时也为 0；近奇异时若 ‖dq_null‖_W 超过 ‖z‖_W 则按比例缩回。
 * 不能用带阻尼的 I − J̄J 当投影：J̄ 带阻尼时它不是投影，会把 z 漏进任务空间，起点离参考构型很远时把任务项顶住（实测 300 个
 * 随机可达目标只有 38 个到位，改成精确投影后 228 个）。
 * 末端误差量纲统一：e = [Δp(m); L·δφ(rad)]，J 的角速度行同样乘 L（orientation_length_m），
 * 即 1 rad 的姿态误差等价于 L 米的位置误差。
 *
 * 约束（Δq 求出后依次施加）：
 *   1. 关节限位：越界的关节钳到边界、移出自由集，其对末端的贡献从 e 里扣掉，剩余关节重新解。最多 max_limit_passes 轮。
 *   2. 零空间项的缩放：先保证任务项 J̄e 不被挤占，零空间项只用剩下的步长余量。
 *   3. 单周期步长：max_step_deg 是每个关节每周期最大转角；超出时整个 Δq 等比缩小，保持方向，线性化才可信。
 *   4. 单周期误差：e 的位置部分和姿态部分各自限幅（max_pos_err_m、max_rot_err_rad），远目标走成一串小步。
 * 参考：opensource/七轴机械臂解算与控制_开源整理.md §2.4（资料 A 的加权 DLS）。 */
/* HOST_IK 的求解方式：解析解一次给出目标关节角；数值解每周期迭代一小步 */
enum class IkMethod : uint8_t {
    Analytic = 0,
    Numeric = 1
};

struct NumericIkParam {
    double damping = 0.03;                  // λ，在 [m; L·rad] 的量纲下
    double orientation_length_m = 0.1;      // L：姿态误差折算成位置误差的等效长度
    double nullspace_gain = 0.02;           // k：每周期向参考构型拉回的比例，0 = 关闭零空间项
    double max_step_deg = 0.2;              // 每关节每周期最大转角，2 ms 周期下约 100 °/s
    double max_pos_err_m = 0.01;            // 单步使用的位置误差上限
    double max_rot_err_rad = 0.05;          // 单步使用的姿态误差上限
    double weight_floor = 1e-3;             // 权重下限，权重 0 会使 W⁻¹ 发散
    double converged_pos_m = 5e-4;          // 位置误差低于此值视为到位
    double converged_rot_rad = 2e-3;        // 姿态误差低于此值视为到位
    int max_limit_passes = 4;               // 限位重解的最大轮数
    double max_lead_deg = 5.0;              // 指令角领先实测角的上限，由调用方（Arm）使用：跟不上时不再往前堆
};

bool IsValidNumericIkParam(const NumericIkParam &param);

struct NumericIkStep {
    JointArray dq_deg{};                    // 本周期的关节增量，已满足限位和步长约束
    double pos_err_m = 0.0;                 // 步前的位置误差范数（未限幅）
    double rot_err_rad = 0.0;               // 步前的姿态误差角（未限幅）
    uint8_t limit_mask = 0;                 // bit i = 第 i 个关节被限位钳住
    bool step_limited = false;              // 步长约束起了作用
    bool converged = false;                 // 步前误差已低于收敛阈值
    bool ok = true;                         // false：输入非法或线性方程不可解，dq 全 0
};

/* 几何 Jacobian（double，行 0..2 线速度、3..5 角速度，基座系），第 i 列对应第 i 个关节。
 * 改进 DH 下第 i 个关节绕坐标系 i+1 的 z 轴转，轴线过该坐标系原点。 */
using Jacobian6x7 = std::array<std::array<double, kJointCount>, 6>;
Jacobian6x7 ComputeJacobian(const FrameArray &frames);

/* 位姿误差：e[0..2] = p_d − p_c，e[3..5] = ½ Σ_k r_c,k × r_d,k（小角度下 ≈ 旋转向量，基座系）。 */
std::array<double, 6> ComputePoseError(const Mat4 &current, const Mat4 &desired);

/* 在 q_deg 处做一步。q_ref_deg 是零空间的参考构型；limit_min/max_deg 是多圈限位，与 q_deg 同一圈。 */
NumericIkStep SolveNumericIkStep(const DhTable &table, const JointArray &q_deg, const Mat4 &target,
                                 const JointArray &q_ref_deg, const IkCostWeights &weights,
                                 const JointArray &limit_min_deg, const JointArray &limit_max_deg,
                                 const NumericIkParam &param);
