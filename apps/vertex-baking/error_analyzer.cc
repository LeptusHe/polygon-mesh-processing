#include "error_analyzer.h"
#include "triangle-sampler.h"

namespace meshlib {

ErrorAnalyzer::ErrorAnalyzer(const Mesh &mesh, const Texture &tex, int sample_num)
    : mesh_(mesh), tex_(tex), sample_num_(sample_num)
{

}

float ErrorAnalyzer::Analyze()
{
    TriangleSampler sampler;
    barycentric_coords_ = sampler.GenerateSamples(sample_num_);

    float sum_error = 0.0f;
    for (const auto fh : mesh_.faces()) {
        sum_error += AnalyzeFaceError(fh);
    }

    return sum_error;
}

float ErrorAnalyzer::AnalyzeFaceError(const Mesh::FaceHandle& fh)
{
    OpenMesh::VertexHandle vhs[3];
    int vertex_index = 0;
    for (const auto vh : mesh_.fv_range(fh)) {
        vhs[vertex_index] = vh;
        vertex_index += 1;
    }

    OpenMesh::Vec2f uvs[3];
    Eigen::Vector3f colors[3];
    for (int i = 0; i < 3; ++ i) {
        const auto uv = mesh_.texcoord2D(vhs[i]);
        const auto color = mesh_.color(vhs[i]);

        uvs[i] = uv;
        colors[i] = Eigen::Vector3f(color[0] / 255.0f, color[1] / 255.0f, color[2] / 255.0f);
    }


    float sum_error = 0.0f;
    for (int i = 0; i < sample_num_; ++ i) {
        auto& barycentric_coord = barycentric_coords_[i];
        const auto uv = barycentric_coord.Interpolate(uvs[0], uvs[1], uvs[2]);
        const auto flip_uv = OpenMesh::Vec2f(uv[0], 1 - uv[1]);

        const auto color_val = barycentric_coord.Interpolate(colors[0], colors[1], colors[2]);
        const auto tex_val = tex_.Sample(flip_uv);

        sum_error += (color_val - tex_val).squaredNorm();
    }
    sum_error /= sample_num_;

    return sum_error;
}

} // namespace meshlib