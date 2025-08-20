#pragma once

#include <Eigen/Eigen>
#include "utils/mesh_utils.h"

namespace meshlib {

class BarycentricCoords {
public:
    BarycentricCoords() = default;
    BarycentricCoords(float x_, float y_, float z_);

    Mesh::TexCoord2D Interpolate(const Mesh::TexCoord2D& v0, const Mesh::TexCoord2D& v1, const Mesh::TexCoord2D& v2);
    Eigen::Vector3f Interpolate(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

public:
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

} // namespace meshlib