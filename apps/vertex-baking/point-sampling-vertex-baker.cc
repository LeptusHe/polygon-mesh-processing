#include "point-sampling-vertex-baker.h"

namespace meshlib {

PointSamplingVertexBaker::PointSamplingVertexBaker(Mesh& mesh, Texture& tex)
    : mesh_(mesh)
    , tex_(tex)
{

}

void PointSamplingVertexBaker::Solve()
{
    mesh_.request_vertex_colors();
    for (const auto vh : mesh_.vertices()) {
        const auto uv = mesh_.texcoord2D(vh);

        const auto flip_uv = OpenMesh::Vec2f(uv[0], 1.0f - uv[1]);
        auto val = tex_.Sample(flip_uv);

        Mesh::Color color(val.x() * 255.0f, val.y() * 255.0f, val.z() * 255.0f);
        mesh_.set_color(vh, color);
    }
}

} // namespace meshlib