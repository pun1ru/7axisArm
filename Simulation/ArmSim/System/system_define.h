#ifndef SYSTEM_DEFINE_H
#define SYSTEM_DEFINE_H

#include <stdint.h>

/* 任务周期，名字和实机 system_define.h 一致
 * 仿真里没有 FreeRTOS，System_Run 用一个单线程调度器按这些周期跑 */
#define TASK_ARM_PERIOD_MS            2U
#define TASK_PLANT_PERIOD_MS          2U      /* 必须等于 MJCF 的 timestep */
#define TASK_MONITOR_PERIOD_MS        100U
#define TASK_HOST_PERIOD_MS           2U

#define SIM_DEFAULT_DURATION_S        8.0

/* 上位机链路：JOINT_STATE 每 HOST_STATE_DIVIDER 个 Host 周期上报一次（2 ms * 5 = 100 Hz），
 * 数值与实机 system_define.h 相同。链路是 pty，这里给它一个固定名字的软链接。 */
#define HOST_STATE_DIVIDER            5U
#define HOST_LINK_PATH                "/tmp/armsim-host"

/* ---- 仿真专用的电机参数，不属于 URDF ----
 * 关节反射转子惯量 + 粘性阻尼。DIY/robot.xml 里没有这两项，远端关节惯量
 * 只有 1.5e-5 kg*m^2，纯力矩控制会抖；加上后与实机手感接近。
 * 置 0 就是“纯 MJCF”。 */
#define SIM_JOINT_ARMATURE            1.0e-3  /* kg*m^2 */
#define SIM_JOINT_DAMPING             1.5    /* N*m*s/rad */
#define SIM_JOINT_FRICTIONLOSS        0.15   /* N*m */
#define SIM_TORQUE_LIMIT_NM           20.0    /* 单关节安全限幅 */

/* CMake 会把它覆盖成源码树里的绝对路径 */
#ifndef ARMSIM_DEFAULT_XML
#define ARMSIM_DEFAULT_XML            "../mujoco/DIY/robot.xml"
#endif

#endif
