#pragma once

#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "utils/mesh_utils.h"
#include "geometry/barycentric-coord.h"
#include "texture/texture.h"
#include "vertex-baker.h"

namespace meshlib {

class LeastSquaresVertexBaker : public VertexBaker {
public:
    struct Options {
        int sample_num = 128;
        bool enable_random_sample = false;
        float regularization_factor = 0.1f;
        bool enable_edge_regularization = false;
        bool debug_integral_method = false;
    };

private:
    struct VertexInfo {
        OpenMesh::VertexHandle vh;
        Eigen::Vector3f pos;
        Eigen::Vector3f gradient;
    };

public:
    LeastSquaresVertexBaker(Mesh& mesh, Texture& tex, const Options& options);
    void Solve() override;

private:
    void Init();
    void GenerateSamplingSamples(int sample_num);
    void CalculateCoefficientMatrix();
    void CalculateConstantVector();
    Eigen::Vector3f CalculateConstantFactor(Mesh::FaceHandle fh, Mesh::VertexHandle v);
    void SolveLinerEquation();
    std::vector<Eigen::Triplet<float>> BuildEdgeRegularizationMatrix();
    void ComputeVertexGradientInTriangle(const OpenMesh::FaceHandle& fh, float& triangle_area, VertexInfo *vertex_infos);
    float ConvertToColorValue(float v);

private:
    OpenMesh::VProp<float> v_prop_;
    OpenMesh::EProp<float> e_prop_;
    OpenMesh::VProp<Eigen::Vector3f> v_constant_prop_;
    Mesh& mesh_;
    std::vector<BarycentricCoords> barycentric_coords_;
    Texture tex_;
    Options options_;
};

} // namespace meshlib