#pragma once

#include <algorithm>
#include <string>

#include "mujoco_world.h"

/* 一个关节执行器：读位置/速度，写力（转动关节 rad、rad/s、N*m；滑动关节 m、m/s、N）。
 * 仿真里电机和关节同轴、理想响应，所以不需要电机状态机、控制模式和 CAN 帧。
 * 方向和零点由 Arm 处理。每个执行器自带出力上限，对应 URDF 的 effort。 */
class JointActuator {
public:
    JointActuator(MujocoWorld &world, const std::string &joint_name, double effort_limit)
        : world_(&world), handle_(world.BindJoint(joint_name)), effort_limit_(effort_limit)
    {
    }

    double position() const { return world_->JointPosition(handle_); }
    double velocity() const { return world_->JointVelocity(handle_); }
    /* 上一次写进物理引擎的力（限幅之后） */
    double effort() const { return effort_; }

    void SetEffort(double effort)
    {
        effort_ = std::clamp(effort == effort ? effort : 0.0, -effort_limit_, effort_limit_);
        world_->SetJointForce(handle_, effort_);
    }

private:
    MujocoWorld *world_;
    int handle_;
    double effort_limit_;
    double effort_ = 0.0;
};
