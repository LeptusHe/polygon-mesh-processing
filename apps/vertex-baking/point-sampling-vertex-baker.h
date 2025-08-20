#pragma once

#include "utils/mesh_utils.h"
#include "geometry/barycentric-coord.h"
#include "texture/texture.h"
#include "vertex-baker.h"

namespace meshlib {

class PointSamplingVertexBaker : public VertexBaker {
public:
    PointSamplingVertexBaker(Mesh& mesh, Texture& tex);
    void Solve() override;

private:
    Mesh& mesh_;
    Texture tex_;

};

} // namespace meshlib