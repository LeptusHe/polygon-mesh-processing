#pragma once

#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "utils/mesh_utils.h"
#include "geometry/barycentric-coord.h"
#include "texture/texture.h"
#include "vertex-baker.h"

namespace meshlib {

class LeastSquaresVertexBaker : public VertexBaker {
public:
    LeastSquaresVertexBaker(Mesh& mesh, Texture& tex);
    void Solve() override;

private:
    void Init();
    void CalculateCoefficientMatrix();
    void CalculateConstantVector();
    Eigen::Vector3f CalculateConstantFactor(Mesh::FaceHandle fh);
    void SolveLinerEquation();
    float ConvertToColorValue(float v);

private:
    OpenMesh::VProp<float> v_prop_;
    OpenMesh::EProp<float> e_prop_;
    OpenMesh::VProp<Eigen::Vector3f> v_constant_prop_;
    Mesh& mesh_;
    std::vector<BarycentricCoords> barycentric_coords_;
    Texture tex_;
};

} // namespace meshlib