#pragma once

#include "mujoco_world.h"

/* 一个关节执行器：读角度/速度，写力矩。
 * 仿真里电机和关节同轴、理想响应，所以不需要电机状态机、控制模式和 CAN 帧。
 * 这一层说的都是关节轴上的量（rad、rad/s、N*m），方向和零点由 Arm 处理。 */
class JointActuator {
public:
    JointActuator(MujocoWorld &world, int joint) : world_(&world), joint_(joint) {}

    double angle_rad() const { return world_->JointAngleRad(joint_); }
    double velocity_rads() const { return world_->JointVelocityRad(joint_); }
    /* 上一次写进物理引擎的力矩（限幅之后） */
    double torque_nm() const { return torque_nm_; }

    void SetTorque(double torque_nm) { torque_nm_ = world_->SetJointTorque(joint_, torque_nm); }

private:
    MujocoWorld *world_;
    int joint_;
    double torque_nm_ = 0.0;
};
