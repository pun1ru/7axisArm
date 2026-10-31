/* 主机端测试驱动：编译同一份 arm_dynamics.c / dh.c，从 stdin 读位姿，输出 C(float32) 算出的重力矩
 * 输入：  第 1 行 n；第 2 行 28 个 pi；第 3 行 12 个 beta；第 4 行 g
 *         之后 n 行，每行 7 个 q(deg) 和 7 行 DH 里的 d（不用，DH 由第 5 行起给定）
 * 为简单，DH 表也从 stdin 读：紧接 g 之后 7 行，每行 alpha a d theta_offset
 * 输出：每个位姿一行，14 个数：前 7 个 pi 版本，后 7 个 beta 版本，%.9g
 * 另外 argv[1]=="model" 时走 ArmDynamics_GravityModel，检查使能 / NaN / 空表行为
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "arm_dynamics.h"

int main(int argc, char **argv)
{
    int n;
    float pi[ARM_GRAVITY_PI_DIM], beta[ARM_GRAVITY_BETA_DIM], g;
    DH_Param_t dh[ARM_GRAVITY_DOF];

    if (scanf("%d", &n) != 1) return 2;
    for (unsigned i = 0; i < ARM_GRAVITY_PI_DIM; ++i) if (scanf("%f", &pi[i]) != 1) return 2;
    for (unsigned i = 0; i < ARM_GRAVITY_BETA_DIM; ++i) if (scanf("%f", &beta[i]) != 1) return 2;
    if (scanf("%f", &g) != 1) return 2;
    for (unsigned i = 0; i < ARM_GRAVITY_DOF; ++i)
        if (scanf("%f %f %f %f", &dh[i].alpha_deg, &dh[i].a_m, &dh[i].d_m, &dh[i].theta_offset_deg) != 4) return 2;

    if (argc > 1 && strcmp(argv[1], "cmd") == 0)
    {
        /* 每行：model[7] valid sign mask limit[7]，输出 cmd[7] */
        for (int k = 0; k < n; ++k)
        {
            float model[ARM_GRAVITY_DOF], limit[ARM_GRAVITY_DOF], cmd[ARM_GRAVITY_DOF];
            int valid, sign, mask;
            for (unsigned i = 0; i < ARM_GRAVITY_DOF; ++i) if (scanf("%f", &model[i]) != 1) return 2;
            if (scanf("%d %d %d", &valid, &sign, &mask) != 3) return 2;
            for (unsigned i = 0; i < ARM_GRAVITY_DOF; ++i) if (scanf("%f", &limit[i]) != 1) return 2;
            for (unsigned i = 0; i < ARM_GRAVITY_DOF; ++i) cmd[i] = 123.0f;
            ArmDynamics_GravityCommand(model, valid != 0, (int8_t)sign, limit, (uint8_t)mask, cmd);
            for (unsigned i = 0; i < ARM_GRAVITY_DOF; ++i) printf("%.9g ", cmd[i]);
            printf("\n");
        }
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "model") == 0)
    {
        /* 每行：q[7] 以及 enable use_beta pi_null beta_null g_in，输出 ret 与 tau[7] */
        for (int k = 0; k < n; ++k)
        {
            float q[ARM_GRAVITY_DOF], tau[ARM_GRAVITY_DOF];
            int en, ub, pn, bn;
            float gi;
            ArmGravityParam_t p;
            for (unsigned i = 0; i < ARM_GRAVITY_DOF; ++i) if (scanf("%f", &q[i]) != 1) return 2;
            if (scanf("%d %d %d %d %f", &en, &ub, &pn, &bn, &gi) != 5) return 2;
            memset(&p, 0, sizeof(p));
            p.enable = (uint8_t)en; p.use_beta = (uint8_t)ub; p.g = gi;
            p.pi = pn ? NULL : pi; p.beta = bn ? NULL : beta;
            for (unsigned i = 0; i < ARM_GRAVITY_DOF; ++i) tau[i] = 123.0f;   /* 检查失败路径是否清零 */
            int ret = ArmDynamics_GravityModel(&p, dh, q, tau);
            printf("%d", ret);
            for (unsigned i = 0; i < ARM_GRAVITY_DOF; ++i) printf(" %.9g", tau[i]);
            printf("\n");
        }
        return 0;
    }

    for (int k = 0; k < n; ++k)
    {
        float q[ARM_GRAVITY_DOF], t1[ARM_GRAVITY_DOF], t2[ARM_GRAVITY_DOF];
        for (unsigned i = 0; i < ARM_GRAVITY_DOF; ++i) if (scanf("%f", &q[i]) != 1) return 2;
        ArmDynamics_GravityTorquePi(dh, q, pi, g, t1);
        ArmDynamics_GravityTorqueBeta(dh, q, beta, g, t2);
        for (unsigned i = 0; i < ARM_GRAVITY_DOF; ++i) printf("%.9g ", t1[i]);
        for (unsigned i = 0; i < ARM_GRAVITY_DOF; ++i) printf("%.9g ", t2[i]);
        printf("\n");
    }
    return 0;
}
