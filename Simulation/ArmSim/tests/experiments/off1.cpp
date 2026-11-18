#include "common.h"
int main(int, char **argv)
{
    const float v[6] = {std::stof(argv[1]), std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5]), std::stof(argv[6])};
    const Mat4 t = TransformFromXyzRpy({v[0], v[1], v[2]}, {v[3] * kDegToRad, v[4] * kDegToRad, v[5] * kDegToRad});
    NumericIkParam p;
    const NumRun r = RunNumeric(JointArray{}, t, IkCostWeights{}, p, 8000);
    std::printf("offline numeric: conv=%d cycles=%d (%.2fs) pos=%.2e m rot=%.3f deg\n q=", r.converged, r.cycles, r.cycles * 0.002, r.pos, r.rot_deg);
    for (float q : r.q) std::printf("%.1f ", q);
    std::printf("\n");
}
