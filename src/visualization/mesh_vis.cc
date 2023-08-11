#include "mesh_vis.h"

namespace meshlib {

void ColorBoundaryVertex(Mesh& mesh)
{
    for (auto vh : mesh.vertices()) {
        if (vh.is_boundary()) {
            mesh.set_color(vh, {1, 0, 0});
        } else {
            mesh.set_color(vh, {1, 1, 1});
        }
    }
}

} // namespace meshlib