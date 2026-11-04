#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdbool.h>

#include "arm.h"
#include "host.h"
#include "sim_plant.h"

/* System 层：MuJoCo 的启动、实例创建和任务调度。
 * 相当于实机的 App 层 + BSP：加载模型、按顺序初始化、每 2 ms 跑一次任务。 */

extern SimPlant_t simPlant;   /* 被控对象 */
extern Arm_t armInstance;     /* 七轴臂对象，模块层唯一实例 */
extern Host_t hostInstance;   /* 上位机协议桥，仅 host_mode 下工作 */

/* host_mode=false：跑内置的脚本演示。
 * host_mode=true ：开伪串口等 Host/armhost 连接，臂直接进入 HOST 状态，由上位机给目标 */
bool System_Init(const char *xml_path, bool host_mode);
/* 跑 duration_s 秒；realtime=false 时尽快算完，不做 sleep。host_mode 下强制实时 */
void System_Run(double duration_s, bool realtime);
/* 可在信号处理函数里调用，让 System_Run 在当前周期结束后返回 */
void System_Stop(void);
void System_Deinit(void);

#endif
