#include "ik_cost.h"

#include <cmath>
#include <limits>

bool IsValidIkCostWeights(const IkCostWeights &weights)
{
    float sum = 0.0f;

    for (float weight : weights.weight)
    {
        if (!std::isfinite(weight) || weight < 0.0f)
        {
            return false;
        }
        sum += weight;
    }
    return sum > 0.0f;
}

float ComputeJointMotionCost(const JointArray &candidate_deg, const JointArray &previous_deg,
                             const IkCostWeights &weights)
{
    float cost = 0.0f;

    for (int i = 0; i < kJointCount; ++i)
    {
        const float weight = weights.weight[static_cast<size_t>(i)];
        if (!std::isfinite(candidate_deg[static_cast<size_t>(i)]) ||
            !std::isfinite(previous_deg[static_cast<size_t>(i)]) ||
            !std::isfinite(weight) || weight < 0.0f)
        {
            return std::numeric_limits<float>::infinity();
        }
        const float delta_deg =
            candidate_deg[static_cast<size_t>(i)] - previous_deg[static_cast<size_t>(i)];
        cost += weight * delta_deg * delta_deg;
    }
    return cost;
}
