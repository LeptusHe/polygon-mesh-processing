#include "simplification.h"

namespace meshlib {

bool Collinear(Mesh& mesh, Mesh::HalfedgeHandle cur, Mesh::HalfedgeHandle next, float& cos_theta)
{
    auto v0 = mesh.from_vertex_handle(cur);
    auto v1 = mesh.to_vertex_handle(cur);
    auto v2 = mesh.to_vertex_handle(next);

    auto p0 = mesh.point(v0);
    auto p1 = mesh.point(v1);
    auto p2 = mesh.point(v2);

    auto l1 = (p0 - p1).normalized();
    auto l2 = (p2 - p1).normalized();

    cos_theta = l1.dot(l2);
    return std::abs(cos_theta + 1) < 1e-3;
}

bool CollapseBoundaryEdge(Mesh& mesh)
{
    Mesh::HalfedgeHandle cur, next;
    bool found = false;

    for (auto hf : mesh.halfedges()) {
        cur = hf;

        if (!mesh.is_boundary(cur))
            continue;

        next = mesh.next_halfedge_handle(cur);
        if (!next.is_valid() || !mesh.is_boundary(next))
            continue;

        float cos_theta = 0;
        if (!Collinear(mesh, cur, next, cos_theta))
            continue;

        if (!mesh.is_collapse_ok(next))
            continue;

        found = true;
        break;
    }

    if (found) {
        //auto v1 = mesh.to_vertex_handle(cur);
        //auto v2 = mesh.to_vertex_handle(next);

        //auto p2 = mesh.point(v2);
        //mesh.set_point(v1, p2);

        auto next_op = mesh.opposite_halfedge_handle(next);
        auto fh = mesh.face_handle(next_op);
        //mesh.delete_face(fh);

        if (mesh.is_collapse_ok(next)) {
            mesh.collapse(next);
        } else {
            std::cout << "failed to collapse half edge" << std::endl;
        }

        mesh.garbage_collection();
    } else {
        std::cout << "failed to find" << std::endl;
    }
    return found;
}

} // namespace meshlib