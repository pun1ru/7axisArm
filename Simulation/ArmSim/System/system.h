#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdbool.h>

#include "arm.h"
#include "sim_plant.h"

/* System 层：MuJoCo 的启动、实例创建和任务调度。
 * 相当于实机的 App 层 + BSP：加载模型、按顺序初始化、每 2 ms 跑一次任务。 */

extern SimPlant_t simPlant;   /* 被控对象 */
extern Arm_t armInstance;     /* 七轴臂对象，模块层唯一实例 */

bool System_Init(const char *xml_path);
/* 跑 duration_s 秒；realtime=false 时尽快算完，不做 sleep */
void System_Run(double duration_s, bool realtime);
void System_Deinit(void);

#endif
