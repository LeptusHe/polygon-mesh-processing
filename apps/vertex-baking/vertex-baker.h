#pragma once

#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "utils/mesh_utils.h"
#include "geometry/barycentric-coord.h"

namespace meshlib {

class VertexBaker {
public:
    explicit VertexBaker(Mesh& mesh);
    void Solve();

private:
    void Init();
    void CalculateCoefficientMatrix();
    void CalculateConstantVector();

private:
    OpenMesh::VProp<float> v_prop_;
    OpenMesh::EProp<float> e_prop_;
    Mesh& mesh_;
    std::vector<BarycentricCoords> barycentric_coords_;
};

} // namespace meshlib