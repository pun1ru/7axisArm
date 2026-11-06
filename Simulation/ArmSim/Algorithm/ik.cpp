#include "ik.h"

#include <cmath>

namespace {

constexpr double kPiD = 3.14159265358979323846;
constexpr double kSingularEps = 1e-6;     // sinθ 低于此值按奇异处理
constexpr double kReachEps = 1e-5;        // cosθ4 允许超出 [-1, 1] 的容差
constexpr double kMinLen = 1e-9;
constexpr double kRefEps = 1e-9;          // 参考构型里 |cosφ| 逼近 1 的容差

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;   // 行主序

Vec3 Add(const Vec3 &a, const Vec3 &b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
Vec3 Sub(const Vec3 &a, const Vec3 &b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
Vec3 Scale(const Vec3 &a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double Dot(const Vec3 &a, const Vec3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
double Norm(const Vec3 &a) { return std::sqrt(Dot(a, a)); }

Vec3 Cross(const Vec3 &a, const Vec3 &b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

Vec3 Col(const Mat3 &R, int j) { return {R[0][j], R[1][j], R[2][j]}; }

Mat3 FromColumns(const Vec3 &x, const Vec3 &y, const Vec3 &z)
{
    return {{{x[0], y[0], z[0]}, {x[1], y[1], z[1]}, {x[2], y[2], z[2]}}};
}

Mat3 Mul(const Mat3 &A, const Mat3 &B)
{
    Mat3 C{};

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
        }
    }
    return C;
}

Mat3 Transpose(const Mat3 &A)
{
    Mat3 T{};

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            T[i][j] = A[j][i];
        }
    }
    return T;
}

Mat3 RotZ(double a)
{
    const double c = std::cos(a), s = std::sin(a);
    return {{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

const Mat3 kRx90 = {{{1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}}};

/* 罗德里格斯：绕单位向量 k 转 angle */
Mat3 Rodrigues(const Vec3 &k, double angle)
{
    const double c = std::cos(angle), s = std::sin(angle), v = 1.0 - c;
    return {{{c + k[0] * k[0] * v,        k[0] * k[1] * v - k[2] * s, k[0] * k[2] * v + k[1] * s},
             {k[1] * k[0] * v + k[2] * s, c + k[1] * k[1] * v,        k[1] * k[2] * v - k[0] * s},
             {k[2] * k[0] * v - k[1] * s, k[2] * k[1] * v + k[0] * s, c + k[2] * k[2] * v}}};
}

Vec3 RotateVec(const Vec3 &axis, double angle, const Vec3 &v)
{
    const Mat3 R = Rodrigues(axis, angle);
    return {R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]};
}

Mat3 RotationOf(const Mat4 &T)
{
    Mat3 R{};

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            R[i][j] = T(i, j);
        }
    }
    return R;
}

Vec3 OriginOf(const Mat4 &T) { return {T(0, 3), T(1, 3), T(2, 3)}; }

double Wrap(double rad) { return std::remainder(rad, 2.0 * kPiD); }   // [-π, π]

bool NearDeg(float value, float target) { return std::fabs(value - target) < 1e-3f; }

/* R = Rz(a)·Rx90·Rz(b)·Rx90·Rz(c) 的分解，lam = sign(sin b)。展开后：
 *   第 3 列 = Rz(a)·(sin b, 0, −cos b)ᵀ：R(0,2) = cos a·sin b，R(1,2) = sin a·sin b，R(2,2) = −cos b
 *   第 3 行 = (sin b·cos c, −sin b·sin c, −cos b)：R(2,0) = sin b·cos c，R(2,1) = −sin b·sin c
 * 乘上 λ = sign(sin b) 去掉 sin b 的符号：
 *   b = atan2(λ·√(R02²+R12²), −R22)，a = atan2(λ·R12, λ·R02)，c = atan2(−λ·R21, λ·R20)
 * 随机角度往返恢复误差 1e-16。b = 0 或 π 时 a 和 c 只有组合量确定，约定 a = 0，c 取剩下的全部。返回是否奇异。 */
bool DecomposeZxzxz(const Mat3 &R, int8_t lam, double &a, double &b, double &c)
{
    const double cb = -R[2][2];
    const double sb = std::hypot(R[0][2], R[1][2]);

    if (sb < kSingularEps)
    {
        a = 0.0;
        b = cb > 0.0 ? 0.0 : kPiD;
        c = std::atan2(-R[1][0], cb * R[0][0]);
        return true;
    }
    const double l = lam < 0 ? -1.0 : 1.0;

    b = std::atan2(l * sb, cb);
    a = std::atan2(l * R[1][2], l * R[0][2]);
    c = std::atan2(-l * R[2][1], l * R[2][0]);
    return false;
}

/* 参考肘轴 y3_ref：y3 ⟂ z0，y3·p̂ = -d4/|p|，取 y3·(z0×p̂) ≥ 0 的那个。
 * 记 h = p̂ 的水平分量单位向量，n = z0×h，则 y3 = cosφ·h + sinφ·n，cosφ = -(d4/|p|)/sinγ，sinγ = |p̂ 水平分量|。 */
bool ReferenceElbowAxis(const SrsParam &prm, const Vec3 &p, Vec3 &y3_ref)
{
    const double len = Norm(p);

    if (len < kMinLen)
    {
        return false;
    }
    const Vec3 u = Scale(p, 1.0 / len);
    const double sin_gamma = std::hypot(u[0], u[1]);

    if (sin_gamma < kArmAngleMinSin)
    {
        return false;
    }
    const double cos_phi = -(prm.elbow_offset_m / len) / sin_gamma;

    if (std::fabs(cos_phi) > 1.0 - kRefEps)
    {
        return false;
    }
    const double sin_phi = std::sqrt(1.0 - cos_phi * cos_phi);
    const Vec3 h = {u[0] / sin_gamma, u[1] / sin_gamma, 0.0};
    const Vec3 n = {-h[1], h[0], 0.0};   // z0 × h

    y3_ref = Add(Scale(h, cos_phi), Scale(n, sin_phi));
    return true;
}

/* R03_ref：已知 y3_ref，由 R03·v = p 求 x3、z3。v 在坐标系 3 里是 (d5·sinθ4, -d4, d3 - d5·cosθ4)。
 * q = p + d4·y3 垂直于 y3，在这个平面里 (z3, x3) 绕 y3 右手排列，q 与 z3 的夹角是 δ = atan2(vx, vz)，
 * 所以 z3 = Rot(y3, -δ)·q̂，x3 = y3 × z3。 */
bool ReferenceR03(const SrsParam &prm, const Vec3 &p, const Vec3 &y3_ref, double theta4, Mat3 &R03_ref)
{
    const Vec3 q = Add(p, Scale(y3_ref, prm.elbow_offset_m));
    const double q_len = Norm(q);

    if (q_len < kMinLen)
    {
        return false;
    }
    const Vec3 q_hat = Scale(q, 1.0 / q_len);
    const double delta = std::atan2(prm.forearm_m * std::sin(theta4),
                                    prm.upper_arm_m - prm.forearm_m * std::cos(theta4));
    const Vec3 z3 = RotateVec(y3_ref, -delta, q_hat);
    const Vec3 x3 = Cross(y3_ref, z3);

    R03_ref = FromColumns(x3, y3_ref, z3);
    return true;
}

}  // namespace

/*--------------------------------------参数--------------------------------------*/
bool MakeSrsParam(const DhTable &t, SrsParam &out)
{
    if (!IsSrsStructure(t) || !NearDeg(t[0].alpha_deg, 0.0f))
    {
        return false;
    }
    for (int k = 1; k < kJointCount; ++k)
    {
        if (!NearDeg(t[k].alpha_deg, 90.0f))
        {
            return false;
        }
    }
    if (!(t[2].d_m > 0.0f) || !(t[4].d_m > 0.0f))
    {
        return false;
    }

    out.shoulder_m = {t[0].a_m, 0.0, t[0].d_m};   // α0 = 0：(a0, -sinα0·d1, cosα0·d1)
    out.upper_arm_m = t[2].d_m;
    out.elbow_offset_m = t[3].d_m;
    out.forearm_m = t[4].d_m;
    out.tool_m = t[6].d_m;
    for (int k = 0; k < kJointCount; ++k)
    {
        out.theta_offset_rad[k] = static_cast<double>(t[k].theta_offset_deg) * kPiD / 180.0;
    }
    return true;
}

/*--------------------------------------肘--------------------------------------*/
bool SolveElbowAngle(const SrsParam &prm, double sw_dist_m, int8_t elbow_branch, double &theta4_rad)
{
    const double d3 = prm.upper_arm_m, d4 = prm.elbow_offset_m, d5 = prm.forearm_m;
    const double c = (d3 * d3 + d5 * d5 + d4 * d4 - sw_dist_m * sw_dist_m) / (2.0 * d3 * d5);

    if (!(std::fabs(c) <= 1.0 + kReachEps))   // 同时挡住 NaN
    {
        return false;
    }
    const double angle = std::acos(c > 1.0 ? 1.0 : (c < -1.0 ? -1.0 : c));

    theta4_rad = elbow_branch < 0 ? -angle : angle;
    return true;
}

/*--------------------------------------臂角--------------------------------------*/
ArmAngle ComputeArmAngle(const SrsParam &prm, const FrameArray &frames)
{
    ArmAngle out;
    const Vec3 s = OriginOf(frames[0]);
    const Vec3 w = OriginOf(frames[4]);
    const Vec3 p = Sub(w, s);
    const double len = Norm(p);
    Vec3 y_ref;

    if (len < kMinLen || !ReferenceElbowAxis(prm, p, y_ref))
    {
        return out;
    }
    const Vec3 u = Scale(p, 1.0 / len);
    const Vec3 y = Col(RotationOf(frames[2]), 1);   // 实际肘轴 y3

    /* y_ref 和 y 去掉沿 u 的分量后，两个向量之间绕 u 的带符号夹角 */
    const double a = Dot(y_ref, u), b = Dot(y, u);
    const double psi = std::atan2(Dot(u, Cross(y_ref, y)), Dot(y_ref, y) - a * b);

    out.valid = true;
    out.psi_deg = static_cast<float>(psi * 180.0 / kPiD);
    return out;
}

/*--------------------------------------逆解--------------------------------------*/
IkResult SolveIk(const SrsParam &prm, const Mat4 &target, const Branch &branch, float psi_deg)
{
    IkResult out;
    const Mat3 R07 = RotationOf(target);
    const Vec3 w = Sub(OriginOf(target), Scale(Col(R07, 2), prm.tool_m));   // 腕心 = 目标 - d7·z7
    const Vec3 p = Sub(w, prm.shoulder_m);
    const double len = Norm(p);
    double theta4 = 0.0;

    if (!SolveElbowAngle(prm, len, branch.elbow, theta4))
    {
        out.status = IkStatus::OutOfReach;
        return out;
    }

    Vec3 y_ref;
    Mat3 R03_ref;

    if (!ReferenceElbowAxis(prm, p, y_ref) || !ReferenceR03(prm, p, y_ref, theta4, R03_ref))
    {
        out.status = IkStatus::RefDegenerate;
        return out;
    }

    /* 臂角：整条手臂绕 S→W 转 ψ */
    const Vec3 u = Scale(p, 1.0 / len);
    const Mat3 R03 = Mul(Rodrigues(u, static_cast<double>(psi_deg) * kPiD / 180.0), R03_ref);

    /* 肩 */
    double th1, th2, th3;
    out.shoulder_singular = DecomposeZxzxz(R03, branch.shoulder, th1, th2, th3);

    /* 腕：Rx90ᵀ·R04ᵀ·R07 = Rz(θ5)·Rx90·Rz(θ6)·Rx90·Rz(θ7)，R04 = R03·Rx90·Rz(θ4) */
    const Mat3 R04 = Mul(Mul(R03, kRx90), RotZ(theta4));
    const Mat3 M = Mul(Mul(Transpose(kRx90), Transpose(R04)), R07);
    double th5, th6, th7;
    out.wrist_singular = DecomposeZxzxz(M, branch.wrist, th5, th6, th7);

    const double theta[kJointCount] = {th1, th2, th3, theta4, th5, th6, th7};

    for (int k = 0; k < kJointCount; ++k)
    {
        out.q_deg[k] = static_cast<float>(Wrap(theta[k] - prm.theta_offset_rad[k]) * 180.0 / kPiD);
    }
    out.status = IkStatus::Ok;
    return out;
}

IkAllBranches SolveIkAllBranches(const SrsParam &prm, const Mat4 &target, float psi_deg)
{
    IkAllBranches all;

    for (int i = 0; i < 8; ++i)
    {
        Branch b;

        b.valid = true;
        b.shoulder = (i & 1) ? -1 : 1;
        b.elbow = (i & 2) ? -1 : 1;
        b.wrist = (i & 4) ? -1 : 1;
        all[i] = SolveIk(prm, target, b, psi_deg);
    }
    return all;
}
