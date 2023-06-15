#include <iostream>
#include <filesystem>
#include <fmt/format.h>
#include <igl/opengl/glfw/Viewer.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include "MeshUtils.h"

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
        std::cout << "failed to flip edge" << std::endl;
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

    // note: 相等时不要flip，很重要
    if (deviation_post >= deviation_pre) {
        if (!mesh.is_flip_ok(edge)) {
            std::cout << "failed to reflip edge" << std::endl;
            return;
        }

        mesh.flip(edge);
    }
}

void EqualizeValences(Mesh& mesh)
{
    for (int i = 0; i < mesh.n_edges(); ++ i) {
        auto edge = mesh.edge_handle(i);

        // Note: 不对边界进行flip处理
        if (mesh.is_boundary(edge))
            continue;

        EqualizeValencesOfEdge(mesh, edge);
    }
}


int main(int argc, char *argv[])
{
    auto path = argc > 1 ? argv[1] : "data/quad.obj";

    Mesh mesh;
    OpenMesh::IO::Options opt;
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


    /*
    std::filesystem::create_directories("data/result/");
    try {

        if (!OpenMesh::IO::write_mesh(mesh, "data/result/openmesh.obj")) {
            std::cerr << fmt::format("failed to write mesh to [{0}]", path) << std::endl;
            return 1;
        } else {
            std::cout << fmt::format("succeed to write mesh to [{0}]", path) << std::endl;
        }
    } catch (std::exception& e) {
        std::cerr << fmt::format("failed to write mesh because [{0}]", e.what()) << std::endl;
        return 1;
    }
     */

    igl::opengl::glfw::Viewer viewer;
    viewer.data().label_size = 5.0f;
    viewer.data().face_based = true;

    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    igl::opengl::glfw::imgui::ImGuiMenu menu;

    viewer.plugins.push_back(&plugin);
    plugin.widgets.push_back(&menu);

    float target_edge_length = 0.5f;
    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        ImGui::InputFloat("target edge length", &target_edge_length);
        if (ImGui::Button("Split Long Edges")) {
            SplitOneLongEdge(mesh, target_edge_length);
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }

        if (ImGui::Button("Equalize Valences")) {
            EqualizeValences(mesh);
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }
    };

    meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);

    //SplitLongEdges(mesh, 0.6f);

    //meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
    viewer.launch();

    return 0;
}