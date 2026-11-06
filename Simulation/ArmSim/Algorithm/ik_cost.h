#pragma once

#include "kinematics.h"

struct IkCostWeights {
    JointArray weight{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
};

bool IsValidIkCostWeights(const IkCostWeights &weights);

/* 加权关节位移代价：Σ weight[i] · (candidate[i] - previous[i])²，角度单位 deg。
 * candidate 和 previous 都是已映射到当前圈的多圈关节目标，差值不做环绕。 */
float ComputeJointMotionCost(const JointArray &candidate_deg, const JointArray &previous_deg,
                             const IkCostWeights &weights);
