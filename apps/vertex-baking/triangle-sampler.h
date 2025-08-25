#pragma once
#include "geometry/barycentric-coord.h"

namespace meshlib {

class TriangleSampler {
public:
    std::vector<BarycentricCoords> GenerateSamples(int sample_num);
};

}  // namespace meshlib