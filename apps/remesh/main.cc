#include <iostream>
#include <fmt/format.h>
#include <igl/opengl/glfw/Viewer.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <imgui.h>
#include "utils/mesh_utils.h"
#include "remesh/remesher.h"
#include <spdlog/spdlog.h>

#include <boost/bimap.hpp>
#include <boost/bimap/set_of.hpp>
#include <boost/bimap/multiset_of.hpp>

#include <CGAL/polygon_mesh_processing/remesh.h>
#include <igl/file_dialog_open.h>

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;

#define ENABLE_DEBUG_LOG 1

void PrintMeshInfo(const Mesh& mesh, const std::string& header = "")
{
    if (!header.empty()) {
        std::cout << header << std::endl;
    }

    std::cout << fmt::format("vertex count: {}\n", mesh.n_vertices());
    std::cout << fmt::format("face count: {}\n", mesh.n_faces());
    std::cout << fmt::format("edge count: {}\n", mesh.n_edges());
    std::cout << fmt::format("half edge count: {}\n\n", mesh.n_halfedges());
}

bool SplitOneLongEdge(Mesh& mesh, float target_edge_length)
{
    for (auto edge: mesh.edges()) {
        if (!edge.is_valid()) {
            spdlog::error("invalid edge for collapse");
            continue;
        }

        if (mesh.calc_edge_length(edge) > target_edge_length) {
            auto mid_point = mesh.calc_edge_midpoint(edge);
            mesh.split(edge, mid_point);
            return true;
        }
    }
    return false;
}

void SplitLongEdges(Mesh& mesh, float target_edge_length)
{
    using Bimap = boost::bimap<
            boost::bimaps::set_of<OpenMesh::EdgeHandle>,
            boost::bimaps::multiset_of<float, std::greater<>>>;
    using long_edge = Bimap::value_type;

    float squared_len_high = target_edge_length * target_edge_length;
    Bimap long_edges;
    for (auto eh : mesh.edges()) {
        float squared_len = mesh.calc_edge_sqr_length(eh);
        if (squared_len > squared_len_high) {
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
    return;

    while (true) {
        if (!SplitOneLongEdge(mesh, target_edge_length)) {
            break;
        }
    }
}

bool CollapseOneShortEdge(Mesh& mesh, float short_len_threshold, float long_len_threshold)
{
    OpenMesh::EdgeHandle e;
    for (auto edge : mesh.edges()) {
        if (!edge.is_valid()) {
            spdlog::error("invalid edge for collapse");
            continue;
        }

        if (edge.is_boundary())
            continue;


        auto v0 = edge.vertex(0);
        auto v1 = edge.vertex(1);
        if (v0.is_boundary() || v1.is_boundary())
            continue;

        if (mesh.calc_edge_length(edge) < short_len_threshold) {
            e = edge;
            break;
        }
    }

    if (!e.is_valid()) {
        return false;
    }

    auto hf = mesh.halfedge_handle(e, 0);
    auto v0 = mesh.from_vertex_handle(hf);
    auto v1 = mesh.to_vertex_handle(hf);

    bool collapse_ok = true;
    auto p1 = mesh.point(v1);
    for (auto vh : mesh.vv_range(v0)) {
        auto p0 = mesh.point(vh);
        if ((p0 - p1).norm()  > long_len_threshold) {
            collapse_ok = false;
            break;
        }
    }

    if (collapse_ok && mesh.is_collapse_ok(hf)) {
        mesh.collapse(hf);
        return true;
    }
    return false;
}

void CollapseShortEdge(Mesh& mesh, float short_len_threshold, float long_len_threshold)
{
    while (true) {
        if (!CollapseOneShortEdge(mesh, short_len_threshold, long_len_threshold)) {
            break;
        }
    }
}

bool CollapseHalfEdge(Mesh& mesh, const OpenMesh::HalfedgeHandle& he, float long_edge_sqr_len)
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

void CollapseShortEdges(Mesh& mesh, float short_edge_len, float long_edge_len)
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

int GetTargetValence(const Mesh& mesh, OpenMesh::VertexHandle vh)
{
    return mesh.is_boundary(vh) ? 4 : 6;
}

void EqualizeValencesOfEdge(Mesh& mesh, OpenMesh::EdgeHandle edge)
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

    if (!mesh.is_flip_ok(edge)) {
#if ENABLE_DEBUG_LOG
        std::cout << "failed to flip edge" << std::endl;
#endif
        return;
    }

    mesh.flip(edge);

    int deviation_post = 0;
    for (int i = 0; i < 4; i++) {
        auto vertex = vertices[i];
        int valence = static_cast<int>(mesh.valence(vertex));
        int target_valence = GetTargetValence(mesh, vertex);
        deviation_post += std::abs(valence - target_valence);
    }

    if (!edge.is_valid()) {
#if ENABLE_DEBUG_LOG
        std::cout << "failed to find halfedge" << std::endl;
#endif
        return;
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

void EqualizeValences(Mesh& mesh)
{
    for (int i = 0; i < mesh.n_edges(); ++ i) {
        auto edge = mesh.edge_handle(i);

        if (!edge.is_valid()) {
            spdlog::error("invalid edge for equalize");
            continue;
        }

        if (!mesh.is_flip_ok(edge) || mesh.status(edge).feature())
            continue;

#if ENABLE_DEBUG_LOG
        auto he = mesh.halfedge_handle(edge, 0);
        /*
        spdlog::info("equalize valences: face: {0}, edge: {1}, from: {2}, to: {3}",
                     mesh.face_handle(he).idx(),
                     edge.idx(),
                     mesh.from_vertex_handle(he).idx(),
                     mesh.to_vertex_handle(he).idx());
        */
#endif

        EqualizeValencesOfEdge(mesh, edge);
    }
}

void TangentialRelaxation(Mesh& mesh)
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

void IncrementalRemesh(Mesh& mesh, OpenMesh::VProp<OpenMesh::Vec3f>& prop, float target_edge_len, int iterative_cnt)
{
    auto long_edge_len = 4.0f / 3.0f * target_edge_len;
    auto short_edge_len = 3.0f / 5.0f * target_edge_len;

    for (int i  = 0; i < iterative_cnt; ++ i) {
        SplitLongEdges(mesh, long_edge_len);
        mesh.garbage_collection();

        CollapseShortEdges(mesh, short_edge_len, long_edge_len);
        mesh.garbage_collection();

        EqualizeValences(mesh);
        mesh.garbage_collection();

        TangentialRelaxation(mesh);
        mesh.garbage_collection();
    }
}

bool ReadMesh(const std::string& path, Mesh& mesh, CMesh& cmesh)
{
    OpenMesh::IO::Options opt = OpenMesh::IO::Options::VertexNormal;
    if (!OpenMesh::IO::read_mesh(mesh, path, opt)) {
        std::cerr << "failed to load mesh from " << path << std::endl;
        return false;
    } else {
        std::cout << fmt::format("succeed to load mesh from [{0}]", path) << std::endl;
        PrintMeshInfo(mesh);
    }

    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();
    mesh.request_halfedge_normals();

    if (!mesh.has_vertex_normals()) {
        std::cerr << "the mesh has no vertex normals" << std::endl;
    }

    mesh.request_vertex_normals();
    mesh.request_face_normals();
    mesh.update_normals();

    if (!pmp::IO::read_polygon_mesh(path, cmesh)) {
        spdlog::error("failed to read mesh from path {}", path);
        return false;
    } else {
        spdlog::info("succeed to read cgal mesh from path {}", path);
    }

    return true;
}


int main(int argc, char *argv[])
{
    auto path = argc > 1 ? argv[1] : "data/quad.obj";

    Mesh mesh;
    CMesh cmesh;
    ReadMesh(path, mesh, cmesh);


    auto prop_points = OpenMesh::VProp<OpenMesh::Vec3f>(mesh, "points");

    igl::opengl::glfw::Viewer viewer;
    viewer.data().label_size = 2.0f;
    viewer.data().face_based = true;

    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    igl::opengl::glfw::imgui::ImGuiMenu menu;

    viewer.plugins.push_back(&plugin);
    plugin.widgets.push_back(&menu);

    bool project = true;
    float target_edge_length = 0.5f;
    int iterative_cnt = 10;
    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        if (ImGui::Button("load mesh")) {
            auto path = igl::file_dialog_open();
            ReadMesh(path, mesh, cmesh);
            prop_points = OpenMesh::VProp<OpenMesh::Vec3f>(mesh, "points");
        }

        ImGui::InputFloat("target edge length", &target_edge_length);
        auto short_edge_length = 4.0f / 5.0f * target_edge_length;
        auto long_edge_length = 4.0f / 3.0f * target_edge_length;

        ImGui::LabelText("short edge length", "%f", short_edge_length);
        ImGui::LabelText("long edge length", "%f", long_edge_length);

        if (ImGui::CollapsingHeader("Remesh Testing")) {
            ImGui::InputInt("iteration count: ", &iterative_cnt);
            auto proxy = mesh;
            if (ImGui::Button("Remesh")) {
                IncrementalRemesh(proxy, prop_points, target_edge_length, iterative_cnt);
                meshlib::MeshUtils::ConvertMeshToViewer(proxy, viewer);
            }

            if (ImGui::Button("remesher")) {
                meshlib::Remesher remesher;
                remesher.IsotropicRemeshing(proxy, target_edge_length, iterative_cnt);
                meshlib::MeshUtils::ConvertMeshToViewer(proxy, viewer);
            }

            if (ImGui::Button("Split One Long Edges")) {
                SplitOneLongEdge(mesh, long_edge_length);
                mesh.garbage_collection();
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            }

            if (ImGui::Button("Split Long Edges")) {
                SplitLongEdges(mesh, long_edge_length);
                mesh.garbage_collection();
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            }

            if (ImGui::Button("Collapse Short Edges")) {
                auto& cloneMesh = mesh;
                CollapseShortEdge(cloneMesh, short_edge_length, long_edge_length);
                cloneMesh.garbage_collection();
                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }

            if (ImGui::Button("openmesh collapse")) {
                auto& cloneMesh = mesh;
                CollapseShortEdges(cloneMesh, short_edge_length, long_edge_length);
                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }

            if (ImGui::Button("Equalize Valences")) {
                EqualizeValences(mesh);
                mesh.garbage_collection();
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            }

            if (ImGui::Button("Tangential Relaxation")) {
                TangentialRelaxation(mesh);
                mesh.garbage_collection();
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            }
        }

        ImGui::Spacing();
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("CGAL Algorithms")) {
            ImGui::Checkbox("enable project", &project);

            if (ImGui::Button("cgal split long")) {
                auto &cloneMesh = cmesh;
                pmp::split_long_edges(cloneMesh.edges(), long_edge_length, cloneMesh);
                cloneMesh.collect_garbage();
                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }

            if (ImGui::Button("cgal collapse")) {
                auto& cloneMesh = cmesh;
                pmp::isotropic_remeshing(CGAL::faces(cloneMesh), static_cast<double>(target_edge_length), cloneMesh,
                                         pmp::parameters::number_of_iterations(1)
                                                 .do_flip(false)
                                                 .do_flip(false)
                                                 .number_of_relaxation_steps(0)
                                                 .do_project(false)
                                                 .protect_constraints(true));
                cloneMesh.collect_garbage();
                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }

            if (ImGui::Button("cgal tangential relaxation")) {
                auto &cloneMesh = cmesh;
                pmp::tangential_relaxation(cloneMesh);
                cloneMesh.collect_garbage();
                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }

            if (ImGui::Button("cgal remesh")) {
                auto cloneMesh = cmesh;
                pmp::isotropic_remeshing(CGAL::faces(cloneMesh), static_cast<double>(target_edge_length), cloneMesh,
                                         pmp::parameters::number_of_iterations(10)
                                                 .protect_constraints(true)
                                                 .do_project(project));
                cloneMesh.collect_garbage();
                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }
        }

        if (ImGui::Button("write mesh")) {
            meshlib::MeshUtils::WriteMesh(mesh, "data/result/remesh.obj");
        }
    };

    meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
    meshlib::MeshUtils::ConvertMeshToViewer(cmesh, viewer);

    //SplitLongEdges(mesh, 0.6f);

    //meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
    viewer.launch();

    return 0;
}