#pragma once

#include <array>
#include <cstdint>

#include "kinematics.h"

/* 3R-R-3R（S-R-S）七轴臂的解析逆解：给定目标位姿、分支 λ2/λ4/λ6 和臂角 ψ，解析算出 7 个关节角。
 * 臂角固定后就是非冗余的 6 自由度问题，所以每个分支恰好一组解，8 个分支共 8 组。
 *
 * 记号（改进 DH，T_{i-1,i} = Rx(α_{i-1})·Tx(a_{i-1})·Rz(θi)·Tz(di)，θi = q[i-1] + theta_offset[i-1]）：
 *   S = 坐标系 1 原点（肩点），E = 坐标系 3 原点（肘点），W = 坐标系 5 原点（腕心）
 *   S→E = d3·z3，E→W = d4·z4 − d5·y4，z4 = −y3
 *   把 S→W 写在坐标系 3 里：v = (d5·sinθ4, −d4, d3 − d5·cosθ4)，所以
 *     |SW|² = d3² + d5² + d4² − 2·d3·d5·cosθ4，并且 y3·(S→W) = −d4（与臂角无关，d4 = 0 时 y3 ⟂ S→W）
 *   d3 = 大臂长，d5 = 小臂长，d4 = 肘轴偏置。θ4 = 0 是折叠，180° 是伸直。
 *
 * 求解顺序：
 *   1. 腕心 W = 目标位置 − d7·z7；p = W − S
 *   2. 肘：θ4 = λ4·acos((d3² + d5² + d4² − |p|²) / (2·d3·d5))
 *   3. 参考构型 R03_ref（ψ = 0）：y3 ⟂ z0 且 y3·p̂ = −d4/|p|，取 y3·(z0×p̂) ≥ 0 的那一个；x3、z3 由 R03_ref·v = p 唯一确定
 *   4. 臂角：R03 = Rot(p̂, ψ)·R03_ref，p̂ = p/|p|。整条手臂绕 S→W 转，腕心不动，所以这就是 1 维自运动
 *   5. 肩：R03 = Rz(θ1)·Rx90·Rz(θ2)·Rx90·Rz(θ3)，按 λ2 分解
 *   6. 腕：Rx90ᵀ·R04ᵀ·R07 = Rz(θ5)·Rx90·Rz(θ6)·Rx90·Rz(θ7)，R04 = R03·Rx90·Rz(θ4)，按 λ6 分解
 * 要求 DH 表满足 MakeSrsParam 的检查。 */

/* 从 DH 表提取的 S-R-S 几何，全部用 double。 */
struct SrsParam {
    std::array<double, 3> shoulder_m{};      // S 在基座系的位置 (a0, 0, d1)，与 q1 无关
    double upper_arm_m = 0.0;                // d3：大臂长 |SE|，要求 > 0
    double elbow_offset_m = 0.0;             // d4：肘轴偏置，可正可负，可为 0
    double forearm_m = 0.0;                  // d5：小臂长 |EW'|，要求 > 0
    double tool_m = 0.0;                     // d7：目标点沿 z7 到腕心的距离，为 0 时目标点就是腕心
    std::array<double, kJointCount> theta_offset_rad{};
};

/* 失败（返回 false）的情况：IsSrsStructure 不满足、α0 ≠ 0、α1..α6 不是 +90°、大臂或小臂长度 ≤ 0 */
bool MakeSrsParam(const DhTable &table, SrsParam &out);

/* 肘部角：由 |SW| 和肘分支算 θ4（DH 角，rad；λ4 = +1 取 [0, π]，-1 取 [-π, 0]）。
 * |SW| 超出 [最小, 最大] 可达距离（cosθ4 超出 [-1, 1] 容差 1e-5）返回 false。 */
bool SolveElbowAngle(const SrsParam &param, double sw_dist_m, int8_t elbow_branch, double &theta4_rad);

/*--------------------------------------臂角 ψ--------------------------------------*
 * ψ = 当前整条手臂绕 S→W 轴（右手定则）相对「参考构型」转过的角度，范围 [-180, 180]。
 * 参考构型 = θ3 = 0 或 180° 的臂平面：肘轴 y3 垂直于基座 z0（R03 的 (2,1) 元 = z0·y3 = −sinθ2·sinθ3 = 0）。
 * y3 的另一个条件 y3·p̂ = −d4/|p| 来自腕心位置，所以满足这两个条件的 y3 恰好有两个（对称于 z0-p̂ 平面）：
 * 取 y3·(z0×p̂) ≥ 0 的那个作为 ψ = 0。θ3 = 0 / 180° 的构型 y3 都垂直于 z0，所以 ψ 落在这两个根上：
 * 一个是 ψ = 0，另一个是 ψ = 2·atan2(sinφ, cosφ·cosγ)，其中 cosφ = −d4/(|p|·sinγ)，γ 是 p 与 z0 的夹角；
 * d4 = 0 时第二个根正好是 180°，d4 ≠ 0 时会偏离（本机 d4 = −2.5 mm，|p| 小时偏得多）。
 * 随机核对：39338 个 θ3 = 0 / 180° 的位姿里 39322 个 ψ 恰好落在这两个根上，另外 16 个最大差 0.23°（未查原因）。
 * 反过来，ψ = 0 时 θ3 一定是 0 或 180°。
 * 论文 §3.3.2 同样用 λ4 决定肘在参考平面哪一侧；论文提取文本里符号丢了，与论文 ψ 的符号没有逐项核对，
 * 不要直接拿论文的表格对数。
 * ψ 无定义（valid = false）：S→W 与 z0 近似平行，参考平面不存在。肘部奇异处 ψ 仍有定义。 */
constexpr double kArmAngleMinSin = 0.02;   // S→W 与 z0 夹角的正弦低于此值（约 1.1°）视为参考平面消失，工程取值

struct ArmAngle {
    bool valid = false;
    float psi_deg = 0.0f;
};
/* 由正运动学的坐标系位姿得到当前臂角，与 SolveIk 的 ψ 定义严格一致（互为逆运算） */
ArmAngle ComputeArmAngle(const SrsParam &param, const FrameArray &frames);

/*--------------------------------------逆解--------------------------------------*/
enum class IkStatus : uint8_t {
    Ok = 0,
    OutOfReach,      // 腕心离肩太远或太近，大小臂够不到
    RefDegenerate    // S→W 与 z0 近似平行，ψ 无定义
};

enum class IkPsiMode : uint8_t {
    Manual = 0,      // 使用请求中的 psi_deg
    Auto = 1         // 在当前分支的采样点中按加权关节位移代价选择 ψ
};

struct IkResult {
    IkStatus status = IkStatus::OutOfReach;
    JointArray q_deg{};                  // 关节角 [-180, 180]，status != Ok 时为 0
    bool shoulder_singular = false;      // sinθ2 ≈ 0：θ1 与 θ3 只有组合量确定，按 θ1 = 0 取值，λ2 不起作用
    bool wrist_singular = false;         // sinθ6 ≈ 0：θ5 与 θ7 同理，按 θ5 = 0 取值，λ6 不起作用
};

/* 单个分支、固定 ψ 的逆解。target 是坐标系 7 在基座系下的位姿（ForwardKinematics 的输出）。不检查关节限位。 */
IkResult SolveIk(const SrsParam &param, const Mat4 &target, const Branch &branch, float psi_deg);

/* 固定 ψ 下 8 个分支各一组，下标 = Branch::index()（bit0 肩、bit1 肘、bit2 腕，置位 = λ 为 -1） */
using IkAllBranches = std::array<IkResult, 8>;
IkAllBranches SolveIkAllBranches(const SrsParam &param, const Mat4 &target, float psi_deg);
