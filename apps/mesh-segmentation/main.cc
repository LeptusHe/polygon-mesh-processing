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

    int clusterCnt = 8;
    int maxIteration = 100;

    cluster.Init(clusterCnt, 1.0f, maxIteration);
    //cluster.Run(clusterCnt, 1., maxIteration);

    //cluster.InitSeed();
    //cluster.RegionGrow();

    igl::opengl::glfw::Viewer viewer;

    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);

    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);

    int totalClusterCnt = clusterCnt + 10; // static_cast<int>(cluster.GetClusterCount());
    ColorSetGenerator colorSetGenerator(totalClusterCnt);
    auto colors = colorSetGenerator.GetColorSet();

    auto set_color_to_mesh = [&]() { ;
        for (auto faceHandle: mesh.faces()) {
            auto clusterId = clusterProp[faceHandle];
            C.row(faceHandle.idx()) = colors[clusterId];

            //auto pos = mesh.calc_face_centroid(faceHandle);
            //auto c = Eigen::Vector3d{pos[0], pos[1], pos[2]};
            //viewer.data().add_label(c, fmt::format("{}", clusterId));
        }

        auto centers = cluster.GetChartCenters();
        for (const auto fh : centers) {
            auto clusterId = clusterProp[fh];
            auto color = colors[clusterId];
            //C.row(fh.idx()) = 0.5 * color; //Eigen::Vector3d{0, 0, 0};
        }
    };


    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        if (ImGui::InputInt("max iteration", &maxIteration)) {
            cluster.Run(clusterCnt, 1, maxIteration);
            set_color_to_mesh();
        }
    };

    viewer.callback_key_down = [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int mod) {
        if (key == 'N') {
            maxIteration += 1;
            cluster.Run(clusterCnt, 1, maxIteration);
            set_color_to_mesh();
        }
        return false;
    };

    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer& viewer) {
        if (cluster.UpdateCluster()) {
            set_color_to_mesh();
            viewer.data().set_colors(C);
        }
        return false;
    };

    //set_color_to_mesh();

    auto center = cluster.FindCenterOfMesh(mesh);
    if (center.is_valid()) {
        C.row(center.idx()) = Eigen::Vector3d{1, 0, 0};
        for (const auto fh : mesh.ff_range(center)) {
            //C.row(fh.idx()) = Eigen::Vector3d{1, 0, 0};
        }
    }

    for (auto faceHandle: mesh.faces()) {
        //C.row(faceHandle.idx()) = Eigen::Vector3d{0, 1, 0};
    }

    for (auto fh : mesh.faces()) {
        if (mesh.is_boundary(fh)) {
            //C.row(fh.idx()) = Eigen::Vector3d{0, 0, 1};
        }
    }

    //viewer.data().show_custom_labels = true;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch();

    return 0;
}