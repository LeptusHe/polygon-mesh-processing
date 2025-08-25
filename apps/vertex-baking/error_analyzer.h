#pragma once
#include "geometry/barycentric-coord.h"
#include "texture/texture.h"
#include "utils/mesh_utils.h"

namespace meshlib {

class ErrorAnalyzer {
public:
    ErrorAnalyzer(const Mesh& mesh, const Texture& tex, int sample_num);
    float Analyze();

private:
    float AnalyzeFaceError(const Mesh::FaceHandle& fh);

private:
    const Mesh& mesh_;
    const Texture &tex_;
    int sample_num_;
    std::vector<BarycentricCoords> barycentric_coords_;
};

} // namespace meshlib