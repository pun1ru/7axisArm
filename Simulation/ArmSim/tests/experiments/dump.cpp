/* 生成仿真闭环测试的目标列表（起点 q=0，分支 5）：
 * R 两者都能到；N 仅数值解能到（当前分支解析解到不了）；A 仅当前分支解析解能到（数值解卡限位）；
 * U 超出臂展 5 cm；X 位置可达、姿态不可行。输出 tag,x,y,z,roll,pitch,yaw（m / deg） */
#include "common.h"

static void Emit(const char *tag, const Mat4 &t)
{
    const Pose p = PoseFromTransform(t);
    std::printf("%s,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n", tag, p.position_m[0], p.position_m[1], p.position_m[2], p.euler_deg[0],
                p.euler_deg[1], p.euler_deg[2]);
}

int main()
{
    const JointArray start{};
    const Branch sb = ComputeBranch(kDh, start);
    const IkCostWeights w{};
    NumericIkParam p;
    std::mt19937 g(2024);
    int cnt[5] = {};
    const int want[5] = {3, 2, 2, 2, 2};
    const char *tags[5] = {"R", "N", "A", "U", "X"};

    for (int it = 0; it < 20000; ++it)
    {
        const JointArray q = RandomQ(g);
        Mat4 t = Fk(q);
        if (std::fabs(PoseFromTransform(t).euler_deg[1]) > 75.0f) continue;
        int kind = -1;
        JointArray tmp{};
        const bool a_same = AnalyticBranch(t, sb, start, tmp);
        const NumRun r = RunNumeric(start, t, w, p, 6000);

        if (a_same && r.converged) kind = 0;
        else if (!a_same && r.converged) kind = 1;
        else if (a_same && !r.converged) kind = 2;
        if (kind == 3 || kind < 0)
        {
            if (cnt[3] < want[3])
            {
                const SrsParam &s = Srs();
                const double rmax = std::sqrt((s.upper_arm_m + s.forearm_m) * (s.upper_arm_m + s.forearm_m) + s.elbow_offset_m * s.elbow_offset_m);
                double u[3], len = 0;
                for (int i = 0; i < 3; ++i)
                {
                    u[i] = t(i, 3) - s.tool_m * t(i, 2) - s.shoulder_m[static_cast<size_t>(i)];
                    len += u[i] * u[i];
                }
                len = std::sqrt(len);
                Mat4 far = t;
                for (int i = 0; i < 3; ++i)
                    far(i, 3) = static_cast<float>(s.shoulder_m[static_cast<size_t>(i)] + u[i] / len * (rmax + 0.05) + s.tool_m * t(i, 2));
                Emit("U", far);
                ++cnt[3];
            }
        }
        if (kind >= 0 && cnt[kind] < want[kind])
        {
            Emit(tags[kind], t);
            ++cnt[kind];
        }
        if (cnt[4] < want[4])
        {
            const Mat4 o = Fk(RandomQ(g));
            Mat4 x = t;
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b) x(a, b) = o(a, b);
            JointArray tmp2{};
            if (std::fabs(PoseFromTransform(x).euler_deg[1]) < 75.0f && !AnalyticAny(x, start, tmp2))
            {
                Emit("X", x);
                ++cnt[4];
            }
        }
        bool done = true;
        for (int k = 0; k < 5; ++k) done = done && cnt[k] >= want[k];
        if (done) break;
    }
    return 0;
}
