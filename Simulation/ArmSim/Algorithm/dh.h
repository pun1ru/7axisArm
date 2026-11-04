#ifndef DH_H
#define DH_H

#include <stdint.h>

/**
 * \brief 改进 DH（Craig）参数，一行 = 坐标系 i 相对 i-1
 * \note  T_{i-1,i} = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(q_i + theta_offset_i) * Tz(d_i)，角度单位 deg
 *        字段顺序与参数表列 | αi-1 | ai-1 | di | θi | 一致；末端固定坐标系那一行 q 恒为 0
 *
 * 与 Real/DIYcontroller/LibSE/Algorithm/Kinematics/dh.h 接口完全一致，
 * 同一份实现可以在仿真和实机上直接复用。
 */
typedef struct {
    float alpha_deg;        ///< αi-1：绕 x_{i-1} 旋转
    float a_m;              ///< ai-1：沿 x_{i-1} 平移
    float d_m;              ///< di：沿 z_i 平移
    float theta_offset_deg; ///< θi 的零位偏置，实际 θi = q_i + theta_offset
} DH_Param_t;

void DH_Transform(const DH_Param_t *param, float q_deg, float T[4][4]);
void DH_LinkTransforms(const DH_Param_t *table, const float *q_deg, uint8_t n, float (*T)[4][4]);
void DH_ForwardKinematics(const DH_Param_t *table, const float *q_deg, uint8_t n, float T[4][4]);

#endif
