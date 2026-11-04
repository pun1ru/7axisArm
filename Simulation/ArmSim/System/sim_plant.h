#ifndef SIM_PLANT_H
#define SIM_PLANT_H

#include <stdbool.h>
#include <stdint.h>

/* MuJoCo 的模型/数据只以指针形式出现在这里，上层不需要 include mujoco.h */
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

#define SIM_PLANT_MAX_JOINTS   16

/* 仿真被控对象（plant）：包一层 MuJoCo，对上层提供“关节角 + 关节力矩”口径的接口。
 * 实机对应 CAN 总线 + 电机 + 机械臂本体，仿真里换成物理引擎。
 *
 * 力矩通过 qfrc_applied 注入，不依赖 XML 里的 actuator，
 * 所以 DIY/robot.xml 这种 nu=0 的模型也能直接跑。 */
typedef struct {
    const char *xml_path;      /* MJCF 路径 */
    double joint_armature;     /* 关节反射转子惯量 [kg*m^2]，0 = 沿用 MJCF */
    double joint_damping;      /* 关节粘性阻尼 [N*m*s/rad]，0 = 沿用 MJCF */
    double joint_frictionloss; /* 关节库仑摩擦 [N*m]，减速器静摩擦的近似，0 = 沿用 MJCF */
    double torque_limit_nm;    /* 单关节力矩安全限幅 [N*m] */
} SimPlantConfig_t;

typedef struct {
    mjModel *model;
    mjData *data;
    int joint_count;
    int qpos_adr[SIM_PLANT_MAX_JOINTS];   /* jointN -> qpos 下标 */
    int dof_adr[SIM_PLANT_MAX_JOINTS];    /* jointN -> qvel / qfrc 下标 */
    double torque_limit_nm;
    int tcp_site_id;
    int warning_count;
    uint32_t step_count;
} SimPlant_t;

/* 加载 XML，按 config 覆盖关节 armature / damping / frictionloss。
 * 失败返回 false，原因见 SimPlant_LastError */
bool SimPlant_Init(SimPlant_t *plant, const SimPlantConfig_t *config);
void SimPlant_Clear(SimPlant_t *plant);
/* 回到 qpos0，时间清零 */
void SimPlant_Reset(SimPlant_t *plant);

/* 每周期：先 ClearTorque，再给各关节 SetJointTorque（内部限幅），最后 Step */
void SimPlant_ClearTorque(SimPlant_t *plant);
void SimPlant_SetJointTorque(SimPlant_t *plant, uint8_t joint, double torque_nm);
void SimPlant_Step(SimPlant_t *plant);

double SimPlant_GetJointAngleRad(const SimPlant_t *plant, uint8_t joint);
double SimPlant_GetJointVelocityRad(const SimPlant_t *plant, uint8_t joint);
double SimPlant_GetTime(const SimPlant_t *plant);
double SimPlant_GetTimestep(const SimPlant_t *plant);
int SimPlant_GetJointCount(const SimPlant_t *plant);
/* 末端 site（名字 "tcp"）在世界系的位置；没有该 site 时返回 false */
bool SimPlant_GetTcpPosition(const SimPlant_t *plant, double pos[3]);
/* MuJoCo 累积的 warning 数（NaN、接触异常等） */
int SimPlant_GetWarningCount(const SimPlant_t *plant);
/* 最近一次加载失败的原因 */
const char *SimPlant_LastError(void);

#endif
