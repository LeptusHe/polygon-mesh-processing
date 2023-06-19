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
#include <spdlog/spdlog.h>

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;

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
        int target_valence = 6;
        if (mesh.is_boundary(vertex)) {
            target_valence = 4;
        }

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
        int target_valence = 6;
        if (mesh.is_boundary(vertex)) {
            target_valence = 4;
        }

        deviation_post += std::abs(valence - target_valence);
    }

    auto hf = mesh.find_halfedge(v2, v3);
    if (!hf.is_valid()) {
#if ENABLE_DEBUG_LOG
        std::cout << "failed to find halfedge" << std::endl;
#endif
        return;
    }

    // note: 相等时不要flip，很重要
    if (deviation_post >= deviation_pre) {
        auto fliped_edge = mesh.edge_handle(hf);
        if (!mesh.is_flip_ok(fliped_edge)) {

#if ENABLE_DEBUG_LOG
            std::cout << "failed to reflip edge" << std::endl;
#endif

            return;
        }

        mesh.flip(fliped_edge);
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

        // Note: 不对边界进行flip处理
        if (mesh.is_boundary(edge))
            continue;

        auto he = mesh.halfedge_handle(edge, 0);

#if ENABLE_LOG
        spdlog::info("equalize valences: face: {0}, edge: {1}, from: {2}, to: {3}",
                     mesh.face_handle(he).idx(),
                     edge.idx(),
                     mesh.from_vertex_handle(he).idx(),
                     mesh.to_vertex_handle(he).idx());
#endif

        EqualizeValencesOfEdge(mesh, edge);
    }
}

void TangentialRelaxation(Mesh& mesh, OpenMesh::VProp<OpenMesh::Vec3f>& prop)
{
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

        prop[vh] = q;
    }

    for (auto vh : mesh.vertices()) {
        // note: 是否要对边界顶点进行处理?
        if (mesh.is_boundary(vh))
            continue;

        auto q = prop[vh];
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

        CollapseShortEdge(mesh, short_edge_len, long_edge_len);
        mesh.garbage_collection();

        EqualizeValences(mesh);
        mesh.garbage_collection();

        TangentialRelaxation(mesh, prop);
        mesh.garbage_collection();
    }
}


int main(int argc, char *argv[])
{
    auto path = argc > 1 ? argv[1] : "data/quad.obj";

    Mesh mesh;
    OpenMesh::IO::Options opt = OpenMesh::IO::Options::VertexNormal;
    if (!OpenMesh::IO::read_mesh(mesh, path, opt)) {
        std::cerr << "failed to load mesh from " << path << std::endl;
        return 1;
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

    auto prop_points = OpenMesh::VProp<OpenMesh::Vec3f>(mesh, "points");

    igl::opengl::glfw::Viewer viewer;
    viewer.data().label_size = 2.0f;
    viewer.data().face_based = true;

    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    igl::opengl::glfw::imgui::ImGuiMenu menu;

    viewer.plugins.push_back(&plugin);
    plugin.widgets.push_back(&menu);

    float target_edge_length = 0.5f;
    int iterative_cnt = 10;
    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        ImGui::InputFloat("target edge length", &target_edge_length);
        auto short_edge_length = 4.0f / 5.0f * target_edge_length;
        auto long_edge_length = 4.0f / 3.0f * target_edge_length;

        ImGui::LabelText("short edge length", "%f", short_edge_length);
        ImGui::LabelText("long edge length", "%f", long_edge_length);

        if (ImGui::Button("Split Long Edges")) {
            SplitOneLongEdge(mesh, long_edge_length);
            mesh.garbage_collection();
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }

        if (ImGui::Button("Collapse Short Edges")) {
            CollapseShortEdge(mesh, short_edge_length, long_edge_length);
            mesh.garbage_collection();
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }

        if (ImGui::Button("Equalize Valences")) {
            EqualizeValences(mesh);
            mesh.garbage_collection();
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }

        if (ImGui::Button("Tangential Relaxation")) {
            TangentialRelaxation(mesh, prop_points);
            mesh.garbage_collection();
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }

        ImGui::Spacing();

        ImGui::InputInt("iteration count: ", &iterative_cnt);
        if (ImGui::Button("Remesh")) {
            IncrementalRemesh(mesh, prop_points, target_edge_length, iterative_cnt);
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }

        if (ImGui::Button("write mesh")) {
            meshlib::MeshUtils::WriteMesh(mesh, "data/result/remesh.obj");
        }
    };

    meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);

    //SplitLongEdges(mesh, 0.6f);

    //meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
    viewer.launch();

    return 0;
}