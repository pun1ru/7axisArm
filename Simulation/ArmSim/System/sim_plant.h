#ifndef SIM_PLANT_H
#define SIM_PLANT_H

#include <stdbool.h>
#include <stdint.h>

/* MuJoCo 的模型/数据只以指针形式出现在这里，上层（Module/App）不需要 include mujoco.h */
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

#define SIM_PLANT_MAX_JOINTS   16

/* 仿真被控对象（BSP 层）：包一层 MuJoCo，对上层提供“关节角 + 力矩”的电机口径接口。
 * 实机对应的是 CAN 总线 + 电机驱动，仿真里换成物理引擎。
 *
 * 力矩通过 qfrc_applied 注入，不依赖 XML 里的 actuator，
 * 所以 DIY/robot.xml 这种 nu=0 的模型也能直接跑。 */
typedef struct {
    const char *xml_path;      /* MJCF 路径 */
    double joint_armature;     /* 关节反射转子惯量 [kg*m^2]，0 = 纯 MJCF */
    double joint_damping;      /* 关节粘性阻尼 [N*m*s/rad]，0 = 纯 MJCF */
    double joint_frictionloss; /* 关节库仑摩擦 [N*m]，减速器静摩擦的近似，0 = 纯 MJCF */
    double torque_limit_nm;    /* 单关节力矩安全限幅 [N*m] */
} SimPlantConfig_t;

typedef struct {
    mjModel *model;
    mjData *data;
    int joint_count;
    int qpos_adr[SIM_PLANT_MAX_JOINTS];
    int dof_adr[SIM_PLANT_MAX_JOINTS];
    double command_torque[SIM_PLANT_MAX_JOINTS];
    double torque_limit_nm;
    int tcp_site_id;
    int warning_count;
    uint32_t step_count;
} SimPlant_t;

/* 加载 XML 并按 config 填写关节 armature / damping。失败返回 false，错误信息见 SimPlant_LastError */
bool SimPlant_Init(SimPlant_t *plant, const SimPlantConfig_t *config);
void SimPlant_Clear(SimPlant_t *plant);
/* 回到 qpos0、时间清零；命令力矩同时清零 */
void SimPlant_Reset(SimPlant_t *plant);
/* 上层每周期把 7 个关节力矩写进来，Step 之前会统一限幅 */
void SimPlant_ClearTorque(SimPlant_t *plant);
void SimPlant_SetJointTorque(SimPlant_t *plant, uint8_t joint, double torque_nm);
/* 推进一个 timestep */
void SimPlant_Step(SimPlant_t *plant);

double SimPlant_GetJointAngleRad(const SimPlant_t *plant, uint8_t joint);
double SimPlant_GetJointVelocityRad(const SimPlant_t *plant, uint8_t joint);
double SimPlant_GetAppliedTorque(const SimPlant_t *plant, uint8_t joint);
double SimPlant_GetTime(const SimPlant_t *plant);
double SimPlant_GetTimestep(const SimPlant_t *plant);
int SimPlant_GetJointCount(const SimPlant_t *plant);
/* 末端 site（名字 "tcp"）在世界系的位置；没有该 site 时返回 false */
bool SimPlant_GetTcpPosition(const SimPlant_t *plant, double pos[3]);
/* 最近一次加载失败的原因 */
const char *SimPlant_LastError(void);
/* MuJoCo 累积的 warning 数（NaN、接触异常等），每周期可读 */
int SimPlant_GetWarningCount(const SimPlant_t *plant);

#endif
