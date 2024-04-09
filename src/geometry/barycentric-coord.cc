#include "barycentric-coord.h"

namespace meshlib {

BarycentricCoords::BarycentricCoords(float x_, float y_, float z_)
    : x(x_), y(y_), z(z_)
{}


Eigen::Vector3f BarycentricCoords::Interpolate(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
    return x * v0 + y * v1 + z * v2;
}

Mesh::TexCoord2D BarycentricCoords::Interpolate(const Mesh::TexCoord2D& v0, const Mesh::TexCoord2D& v1, const Mesh::TexCoord2D& v2)
{
    return x * v0 + y * v1 + z * v2;
}
} // namespace meshlib