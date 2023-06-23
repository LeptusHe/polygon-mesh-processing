#include "remesher.h"

#include <boost/bimap.hpp>
#include <boost/bimap/set_of.hpp>
#include <boost/bimap/multiset_of.hpp>

namespace meshlib {

Remesher::Remesher()
{

}

void Remesher::IsotropicRemeshing(Mesh &mesh, float target_edge_len, int num_iteration)
{
    auto short_edge_len = 3.0f / 5.0f * target_edge_len;
    auto long_edge_len = 4.0f / 3.0f * target_edge_len;

    for (int i = 0; i < num_iteration; ++ i) {
        SplitLongEdges(mesh, long_edge_len);
        CollapseShortEdges(mesh, short_edge_len, long_edge_len);
        EqualizeValences(mesh);
        TangentialRelaxation(mesh);
    }
}

void Remesher::SplitLongEdges(Mesh& mesh, float long_edge_len)
{
    using Bimap = boost::bimap<
            boost::bimaps::set_of<OpenMesh::EdgeHandle>,
            boost::bimaps::multiset_of<float, std::greater<>>>;
    using long_edge = Bimap::value_type;

    const float squared_long_edge_len = long_edge_len * long_edge_len;
    Bimap long_edges;
    for (auto eh : mesh.edges()) {
        float squared_len = mesh.calc_edge_sqr_length(eh);
        if (squared_len > squared_long_edge_len) {
            long_edges.insert(long_edge(eh, squared_len));
        }
    }

    while (!long_edges.empty()) {
        auto e_iter = long_edges.right.begin();
        auto eh = e_iter->second;
        long_edges.right.erase(e_iter);

        auto mid_point = mesh.calc_edge_midpoint(eh);
        mesh.split(eh, mid_point);
    }

    mesh.garbage_collection();
}

void Remesher::CollapseShortEdges(Mesh& mesh, float short_edge_len, float long_edge_len)
{
    const float short_edge_sqr_len = short_edge_len * short_edge_len;
    const float long_edge_sqr_len = long_edge_len * long_edge_len;

    OpenMesh::EPropHandleT<bool> visited;
    auto visited_property_name = "visited-prop";
    if (!mesh.get_property_handle(visited, visited_property_name)) {
        mesh.add_property(visited, visited_property_name);
    }

    for (auto edge : mesh.edges()) {
        mesh.property(visited, edge) = false;
    }

    while (true) {
        bool generate_new_edge = false;

        for (auto iter = mesh.edges_begin(); iter != mesh.edges_end(); ++ iter) {
            if (mesh.property(visited, *iter))
                continue;

            mesh.property(visited, *iter) = true;
            auto sqr_len = mesh.calc_edge_sqr_length(*iter);

            if ((sqr_len >= short_edge_sqr_len) || (sqr_len <= std::numeric_limits<float>::epsilon()))
                continue;

            auto he = mesh.halfedge_handle(*iter, 0);
            auto oe = mesh.halfedge_handle(*iter, 1);

            generate_new_edge = CollapseHalfEdge(mesh, he, long_edge_sqr_len);
            if (!generate_new_edge) {
                generate_new_edge = CollapseHalfEdge(mesh, oe, long_edge_sqr_len);
            }
        }

        if (!generate_new_edge) {
            break;
        }
    }

    mesh.remove_property(visited);
    mesh.garbage_collection();
}

bool Remesher::CollapseHalfEdge(Mesh& mesh, const OpenMesh::HalfedgeHandle& he, float long_edge_sqr_len)
{
    auto v0 = mesh.from_vertex_handle(he);
    auto v1 = mesh.to_vertex_handle(he);
    auto p1 = mesh.point(v1);

    bool collapse_ok = true;
    for (auto vh : mesh.voh_range(v0)) {
        auto sqr_dist = (p1 - mesh.point(mesh.to_vertex_handle(vh))).sqrnorm();

        if ((sqr_dist > long_edge_sqr_len)
            || (mesh.status(mesh.edge_handle(vh)).feature())
            || (mesh.is_boundary(mesh.edge_handle(vh)))) {
            collapse_ok = false;
            break;
        }
    }

    if (collapse_ok && mesh.is_collapse_ok(he)) {
        mesh.collapse(he);
        return true;
    }
    return false;
}

void Remesher::EqualizeValences(Mesh& mesh)
{
    for (int i = 0; i < mesh.n_edges(); ++ i) {
        auto edge = mesh.edge_handle(i);

        if (!edge.is_valid()) {
#if ENABLE_DEBUG_LOG
            spdlog::error("invalid edge for equalize");
#endif
            continue;
        }

        if (!mesh.is_flip_ok(edge) || mesh.status(edge).feature())
            continue;

#if ENABLE_DEBUG_LOG
        auto he = mesh.halfedge_handle(edge, 0);
        spdlog::info("equalize valences: face: {0}, edge: {1}, from: {2}, to: {3}",
                     mesh.face_handle(he).idx(),
                     edge.idx(),
                     mesh.from_vertex_handle(he).idx(),
                     mesh.to_vertex_handle(he).idx());
#endif

        EqualizeValencesOfEdge(mesh, edge);
    }
    mesh.garbage_collection();
}

void Remesher::EqualizeValencesOfEdge(Mesh& mesh, const OpenMesh::EdgeHandle& edge)
{
    auto hf0 = mesh.halfedge_handle(edge, 0);
    auto hf1 = mesh.halfedge_handle(edge, 1);

    auto v0 = mesh.from_vertex_handle(hf0);
    auto v1 = mesh.to_vertex_handle(hf0);

    auto next_hf0 = mesh.next_halfedge_handle(hf0);
    auto v2 = mesh.to_vertex_handle(next_hf0);

    auto next_hf1 = mesh.next_halfedge_handle(hf1);
    auto v3 = mesh.to_vertex_handle(next_hf1);

    auto vertices = new OpenMesh::VertexHandle[4] { v0, v1, v2, v3 };

    int deviation_pre = 0;
    for (int i = 0; i < 4; i++) {
        auto vertex = vertices[i];
        int valence = static_cast<int>(mesh.valence(vertex));
        int target_valence = GetTargetValence(mesh, vertex);
        deviation_pre += std::abs(valence - target_valence);
    }

    mesh.flip(edge);

    int deviation_post = 0;
    for (int i = 0; i < 4; i++) {
        auto vertex = vertices[i];
        int valence = static_cast<int>(mesh.valence(vertex));
        int target_valence = GetTargetValence(mesh, vertex);
        deviation_post += std::abs(valence - target_valence);
    }

    // note: 相等时不要flip，很重要
    if (deviation_post >= deviation_pre) {
        if (!mesh.is_flip_ok(edge)) {
#if ENABLE_DEBUG_LOG
            std::cout << "failed to reflip edge" << std::endl;
#endif
            return;
        }

        mesh.flip(edge);
    }
}

int Remesher::GetTargetValence(const Mesh& mesh, const OpenMesh::VertexHandle& vh)
{
    return mesh.is_boundary(vh) ? 4 : 6;
}

void Remesher::TangentialRelaxation(Mesh& mesh)
{
    OpenMesh::VPropHandleT<OpenMesh::Vec3f> point_prop;
    const auto point_prop_name = "relaxation-point";
    if (!mesh.get_property_handle(point_prop, point_prop_name)) {
        mesh.add_property(point_prop, point_prop_name);
    }

    for (auto vh : mesh.vertices()) {
        if (mesh.is_boundary(vh))
            continue;

        int n = 0;
        OpenMesh::Vec3f q(0, 0, 0);
        for (auto adj_vh : mesh.vv_range(vh)) {
            q += mesh.point(adj_vh);
            n += 1;
        }
        q = q / n;

        mesh.property(point_prop, vh) = q;
    }

    for (auto vh : mesh.vertices()) {
        // note: 是否要对边界顶点进行处理?
        if (mesh.is_boundary(vh))
            continue;

        auto q = mesh.property(point_prop, vh);
        auto p = mesh.point(vh);
        auto normal = mesh.calc_normal(vh);

        auto new_point = q + normal.dot(p - q) * normal;
        mesh.set_point(vh, new_point);
    }
}

} // namespace meshlib