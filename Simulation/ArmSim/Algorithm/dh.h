#ifndef DH_H
#define DH_H

#include <stdint.h>

/* 改进 DH（Craig），一行 = 坐标系 i 相对 i-1，角度 deg，长度 m
 *   T_{i-1,i} = Rx(alpha) * Tx(a) * Rz(q + theta_offset) * Tz(d)
 * 末端固定坐标系那一行 q 恒为 0。与实机 LibSE/Algorithm/Kinematics/dh.h 接口相同。 */
typedef struct {
    float alpha_deg;        ///< αi-1：绕 x_{i-1} 旋转
    float a_m;              ///< ai-1：沿 x_{i-1} 平移
    float d_m;              ///< di：沿 z_i 平移
    float theta_offset_deg; ///< θi 的零位偏置
} DH_Param_t;

void DH_Transform(const DH_Param_t *param, float q_deg, float T[4][4]);
void DH_LinkTransforms(const DH_Param_t *table, const float *q_deg, uint8_t n, float (*T)[4][4]);
void DH_ForwardKinematics(const DH_Param_t *table, const float *q_deg, uint8_t n, float T[4][4]);

#endif
