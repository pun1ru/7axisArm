/* Motor 的主机桩：Arm 只用到 BindCan 与 UpdateFeedbackStatus，两者在这里都是空操作，反馈由测试直接写 */
#include "Motor.h"
bool Motor_BindCan(Motor_t *motor, CanInstance_t *can) { (void)motor; (void)can; return true; }
void Motor_UpdateFeedbackStatus(Motor_t *motor) { (void)motor; }
