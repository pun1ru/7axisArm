#include "sim_plant.h"

#include <mujoco/mujoco.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

static char s_last_error[1024];

/* ---- MuJoCo 日志回调：把引擎 warning 记到 stderr，避免跑飞时只剩一行 "warning" ---- */
static void SimPlant_LogWarning(const char *msg)
{
    fprintf(stderr, "[mujoco] %s\n", msg);
}

const char *SimPlant_LastError(void)
{
    return s_last_error;
}

bool SimPlant_Init(SimPlant_t *plant, const SimPlantConfig_t *config)
{
    char err[1024] = {};

    if (plant == NULL || config == NULL || config->xml_path == NULL)
    {
        snprintf(s_last_error, sizeof(s_last_error), "SimPlant_Init: null argument");
        return false;
    }

    memset(plant, 0, sizeof(*plant));
    plant->tcp_site_id = -1;
    plant->torque_limit_nm = config->torque_limit_nm > 0.0 ? config->torque_limit_nm : 100.0;

    plant->model = mj_loadXML(config->xml_path, NULL, err, sizeof(err));
    if (plant->model == NULL)
    {
        snprintf(s_last_error, sizeof(s_last_error), "load %s failed:\n%s", config->xml_path, err);
        return false;
    }

    /* 关节 armature / damping 是电机侧的模型，不在 URDF 里。
     * 远端关节惯量很小（J6 约 1.5e-5 kg*m^2），不加这两项时力矩控制会抖，
     * 加上后与实机“转子惯量 + 粘性阻尼”的手感接近。数值在 App/system_define.h 里调。 */
    if (config->joint_armature > 0.0)
    {
        for (int i = 0; i < plant->model->nv; ++i)
        {
            plant->model->dof_armature[i] = config->joint_armature;
        }
    }
    if (config->joint_damping > 0.0)
    {
        for (int i = 0; i < plant->model->nv; ++i)
        {
            plant->model->dof_damping[i] = config->joint_damping;
        }
    }
    /* 库仑摩擦：实机减速器静摩擦能“锁住”关节，纯 viscous damping 只会减慢漂移。
     * 远端关节 kp 很小，靠它才能停在重力前馈的残差上。 */
    if (config->joint_frictionloss > 0.0)
    {
        for (int i = 0; i < plant->model->nv; ++i)
        {
            plant->model->dof_frictionloss[i] = config->joint_frictionloss;
        }
    }

    plant->data = mj_makeData(plant->model);
    if (plant->data == NULL)
    {
        snprintf(s_last_error, sizeof(s_last_error), "mj_makeData failed");
        mj_deleteModel(plant->model);
        plant->model = NULL;
        return false;
    }

    /* 约定关节名 joint1..jointN；找不到就认为链结束 */
    for (int i = 0; i < SIM_PLANT_MAX_JOINTS; ++i)
    {
        char name[32];
        int jid;
        int dof;

        snprintf(name, sizeof(name), "joint%d", i + 1);
        jid = mj_name2id(plant->model, mjOBJ_JOINT, name);
        if (jid < 0)
        {
            break;
        }
        dof = plant->model->jnt_dofadr[jid];
        plant->qpos_adr[i] = plant->model->jnt_qposadr[jid];
        plant->dof_adr[i] = dof;
        plant->joint_count = i + 1;
    }
    if (plant->joint_count == 0)
    {
        snprintf(s_last_error, sizeof(s_last_error), "%s has no joint1..jointN", config->xml_path);
        SimPlant_Clear(plant);
        return false;
    }

    plant->tcp_site_id = mj_name2id(plant->model, mjOBJ_SITE, "tcp");

    mju_user_warning = SimPlant_LogWarning;
    mj_forward(plant->model, plant->data);
    return true;
}

void SimPlant_Clear(SimPlant_t *plant)
{
    if (plant == NULL)
    {
        return;
    }
    if (plant->data != NULL)
    {
        mj_deleteData(plant->data);
        plant->data = NULL;
    }
    if (plant->model != NULL)
    {
        mj_deleteModel(plant->model);
        plant->model = NULL;
    }
    plant->joint_count = 0;
}

void SimPlant_Reset(SimPlant_t *plant)
{
    if (plant == NULL || plant->model == NULL || plant->data == NULL)
    {
        return;
    }
    mj_resetData(plant->model, plant->data);
    memset(plant->command_torque, 0, sizeof(plant->command_torque));
    plant->warning_count = 0;
    plant->step_count = 0;
    mj_forward(plant->model, plant->data);
}

void SimPlant_ClearTorque(SimPlant_t *plant)
{
    if (plant == NULL || plant->data == NULL)
    {
        return;
    }
    memset(plant->command_torque, 0, sizeof(plant->command_torque));
    memset(plant->data->qfrc_applied, 0,
           sizeof(mjtNum) * (size_t)plant->model->nv);
}

void SimPlant_SetJointTorque(SimPlant_t *plant, uint8_t joint, double torque_nm)
{
    if (plant == NULL || plant->data == NULL || joint >= plant->joint_count)
    {
        return;
    }
    if (!(torque_nm == torque_nm))   /* NaN */
    {
        torque_nm = 0.0;
    }
    if (torque_nm > plant->torque_limit_nm)
    {
        torque_nm = plant->torque_limit_nm;
    }
    else if (torque_nm < -plant->torque_limit_nm)
    {
        torque_nm = -plant->torque_limit_nm;
    }
    plant->command_torque[joint] = torque_nm;
    plant->data->qfrc_applied[plant->dof_adr[joint]] = torque_nm;
}

void SimPlant_Step(SimPlant_t *plant)
{
    if (plant == NULL || plant->model == NULL || plant->data == NULL)
    {
        return;
    }
    mj_step(plant->model, plant->data);
    plant->step_count++;
    plant->warning_count += plant->data->warning[0].number;
    plant->data->warning[0].number = 0;
}

double SimPlant_GetJointAngleRad(const SimPlant_t *plant, uint8_t joint)
{
    if (plant == NULL || plant->data == NULL || joint >= plant->joint_count)
    {
        return 0.0;
    }
    return plant->data->qpos[plant->qpos_adr[joint]];
}

double SimPlant_GetJointVelocityRad(const SimPlant_t *plant, uint8_t joint)
{
    if (plant == NULL || plant->data == NULL || joint >= plant->joint_count)
    {
        return 0.0;
    }
    return plant->data->qvel[plant->dof_adr[joint]];
}

double SimPlant_GetAppliedTorque(const SimPlant_t *plant, uint8_t joint)
{
    if (plant == NULL || joint >= plant->joint_count)
    {
        return 0.0;
    }
    return plant->command_torque[joint];
}

double SimPlant_GetTime(const SimPlant_t *plant)
{
    return (plant != NULL && plant->data != NULL) ? plant->data->time : 0.0;
}

double SimPlant_GetTimestep(const SimPlant_t *plant)
{
    return (plant != NULL && plant->model != NULL) ? plant->model->opt.timestep : 0.0;
}

int SimPlant_GetJointCount(const SimPlant_t *plant)
{
    return plant != NULL ? plant->joint_count : 0;
}

bool SimPlant_GetTcpPosition(const SimPlant_t *plant, double pos[3])
{
    if (plant == NULL || plant->data == NULL || plant->tcp_site_id < 0 || pos == NULL)
    {
        return false;
    }
    for (int i = 0; i < 3; ++i)
    {
        pos[i] = plant->data->site_xpos[3 * plant->tcp_site_id + i];
    }
    return true;
}

int SimPlant_GetWarningCount(const SimPlant_t *plant)
{
    return plant != NULL ? plant->warning_count : 0;
}
