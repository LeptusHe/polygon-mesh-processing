#include <iostream>
#include <filesystem>
#include <fmt/format.h>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <imgui.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "IterativeCluster.h"
#include "ColorSetGenerator.h"

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

    if (!opt.face_has_normal()) {
        mesh.request_face_normals();
        mesh.update_face_normals();
    }

    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);
    Eigen::MatrixXi F = Eigen::MatrixXi::Zero(mesh.n_faces(), 3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(mesh.n_faces(), 3);

    for (auto vertHandle : mesh.vertices()) {
        auto vert = mesh.point(vertHandle);
        V.row(vertHandle.idx()) = Eigen::Vector3d(vert[0], vert[1], vert[2]);
    }

    for (auto faceHandle : mesh.faces()) {
        auto fvIter = mesh.cfv_iter(faceHandle);
        for (int i = 0; i < 3; ++ i) {
            F(faceHandle.idx(), i) = fvIter->idx();
            ++ fvIter;
        }
    }

    auto clusterProp = OpenMesh::FProp<int>(mesh, "cluster");
    IterativeCluster cluster(mesh, clusterProp);

    int clusterCnt = 3;
    int maxIteration = 1000;
    cluster.Run(clusterCnt, 1, maxIteration);

    //cluster.InitSeed();
    //cluster.RegionGrow();

    igl::opengl::glfw::Viewer viewer;

    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);

    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);

    ColorSetGenerator colorSetGenerator(clusterCnt);
    auto colors = colorSetGenerator.GetColorSet();

    auto set_color_to_mesh = [&]() { ;
        for (auto faceHandle: mesh.faces()) {
            auto clusterId = clusterProp[faceHandle];
            C.row(faceHandle.idx()) = colors[clusterId];

            auto pos = mesh.calc_face_centroid(faceHandle);
            auto c = Eigen::Vector3d{pos[0], pos[1], pos[2]};
            viewer.data().add_label(c, fmt::format("{}", clusterId));
        }
    };


    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        if (ImGui::InputInt("max iteration", &maxIteration)) {
            cluster.Run(clusterCnt, 1, maxIteration);
            set_color_to_mesh();
        }
    };

    set_color_to_mesh();

    viewer.data().label_size = 2;
    viewer.data().show_custom_labels = true;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch();

    return 0;
}