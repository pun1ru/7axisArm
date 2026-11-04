#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>

#define limiter(val, max) ((val) < (-(max)) ? (-(max)) : ((val) > (max) ? (max) : (val)))

/// @brief pid 计算结构体（与 Real/DIYcontroller 的实现保持一致）
typedef struct {
	float kp, ki, kd;
	float cur_error, last_error, last_last_error, sum_error;
	float output;
	float deriv_filtered;   // D 项低通后的值
	float d_alpha;          // D 项低通系数 [0, 1)，0 = 不滤波
	float sum_error_max, output_max;
} PIDStruct;

void PIDReset(PIDStruct *const pid);
void PIDInitialize(PIDStruct *const pid, float kp, float ki, float kd, float d_alpha,
                   float sum_error_max, float output_max);
float PIDUpdate(PIDStruct *const pid, const float error);
float PIDUpdatePrior(PIDStruct pid, const float error);

/*optimize pid compute*/
float PIDUpdateIntergrateSeparation(PIDStruct *const pid, const float error, const float error_limit);
float PIDUpdateIncrement(PIDStruct *const pid, float current_error);

#endif
