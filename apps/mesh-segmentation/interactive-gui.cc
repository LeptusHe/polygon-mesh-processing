#include <filesystem>
#include <fmt/format.h>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <imgui.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "filesys/filesys.h"
#include "iterative-cluster.h"
#include "visualization/color_set_generator.h"

namespace fs = std::filesystem;
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

    float area = 0.0f;
    for (const auto faceHandle : mesh.faces()) {
        area += mesh.calc_face_area(static_cast<Mesh::FaceHandle>(faceHandle));
    }
    std::cout << fmt::format("total area: {}\n", area);
}

void PrintClusterInfo(const IterativeCluster& cluster)
{
    const auto cluster_cnt = cluster.GetClusterCount();


}


int interactive(int argc, char *argv[])
{
    FileManager::AddSearchPath("./");
    FileManager::AddSearchPath("./../../data");

    const auto config_file_name = "data/config.json";
    if (!fs::exists(config_file_name)) {
        spdlog::error("failed to find {} file", config_file_name);
        return  -1;
    }

    std::ifstream fin(config_file_name);
    auto json_data = nlohmann::json::parse(fin);

    const auto file_name = json_data["input-file"].get<std::string>();
    const auto path = FileManager::FindFile(file_name);
    if (path.empty()) {
        spdlog::error("failed to find input mesh file: {0}", file_name);
        return -1;
    }

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

    float area = 0.0f;
    for (auto faceHandle : mesh.faces()) {
        area += mesh.calc_face_area(faceHandle);

        auto fvIter = mesh.cfv_iter(faceHandle);
        for (int i = 0; i < 3; ++ i) {
            F(faceHandle.idx(), i) = fvIter->idx();
            ++ fvIter;
        }
    }
    std::cout << fmt::format("mesh total area: {}\n", area);

    auto clusterProp = OpenMesh::FProp<int>(mesh, "cluster");
    IterativeCluster cluster(mesh, clusterProp);

    auto options_data = json_data["segmentation-options"];

    IterativeCluster::Options options;
    options.maxChartArea = options_data["max-chart-area"].get<float>();
    options.minClusterCnt = options.maxChartArea > 0 ? static_cast<int>(std::ceil(area / options.maxChartArea)) : 1;
    options.maxIterationNum = options_data["max-iteration-num"].get<int>();
    options.normalWeight = options_data["normal-weight"].get<float>();
    options.enableUVbounds = options_data["enable-uv-bounds"].get<bool>();

    auto max_uv_size = Eigen::Vector2f(8.0f, 8.0f);
    max_uv_size.x() = options_data["max-uv-size-x"].get<float>();
    max_uv_size.y() = options_data["max-uv-size-y"].get<float>();
    options.maxUVSize = max_uv_size;

    cluster.Init(options);

    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);

    int totalClusterCnt = options.minClusterCnt + 10; // static_cast<int>(cluster.GetClusterCount());
    ColorSetGenerator colorSetGenerator(totalClusterCnt);
    auto colors = colorSetGenerator.GetColorSet();

    auto set_color_to_mesh = [&]() {
        auto count = cluster.GetClusterCount();
        auto colorGenerator = ColorSetGenerator(count);
        auto colors = colorGenerator.GetColorSet();

        for (auto faceHandle: mesh.faces()) {
            auto clusterId = clusterProp[faceHandle];
            if (clusterId == -1) {
                C.row(faceHandle.idx()) = Eigen::Vector3d{0, 0, 0};
            } else {
                C.row(faceHandle.idx()) = colors[clusterId];
            }

            //auto pos = mesh.calc_face_centroid(faceHandle);
            //auto c = Eigen::Vector3d{pos[0], pos[1], pos[2]};
            //viewer.data().add_label(c, fmt::format("{}", clusterId));
        }

        auto centers = cluster.GetChartCenters();
        for (const auto fh : centers) {
            auto clusterId = clusterProp[fh];
            //auto color = colors[clusterId];
            //C.row(fh.idx()) = 0.5 * color; //Eigen::Vector3d{0, 0, 0};
        }
    };


    int clusterColorIndex = 0;
    int maxIteration = options.maxIterationNum;
    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        if (ImGui::InputInt("max iteration", &maxIteration)) {
            options.maxIterationNum = maxIteration;
            cluster.Run(options);
            set_color_to_mesh();
        }

        if (ImGui::Button("output chart area")) {
            const auto clusterCount = cluster.GetClusterCount();
            std::vector areas(clusterCount, 0.0f);

            for (const auto faceHandle : mesh.faces()) {
                const auto clusterId = clusterProp[faceHandle];
                areas[clusterId] += mesh.calc_face_area(faceHandle);
            }

            float totalArea = 0.0f;
            for (int i = 0; i < clusterCount; ++ i) {
                totalArea += areas[i];
                std::cout << fmt::format("cluster {} area: {}\n", i, areas[i]);
            }
            std::cout << fmt::format("total area: {}\n", totalArea);
        }

        if (ImGui::InputInt("cluster index", &clusterColorIndex)) {
            for (auto faceHandle : mesh.faces()) {
                const auto clusterId = clusterProp[static_cast<Mesh::FaceHandle>(faceHandle)];
                if (clusterId == clusterColorIndex) {
                    C.row(faceHandle.idx()) = Eigen::Vector3d{1, 0, 0};
                } else {
                    C.row(faceHandle.idx()) = Eigen::Vector3d{0, 0, 0};
                }
            }
            viewer.data().set_colors(C);
        }

        if (ImGui::Button("display boundary")) {
            for (auto fh : mesh.faces()) {
                if (mesh.is_boundary(static_cast<Mesh::FaceHandle>(fh))) {
                    C.row(fh.idx()) = Eigen::Vector3d{1, 0, 0};
                } else {
                    C.row(fh.idx()) = Eigen::Vector3d{0, 0, 0};
                }
            }
            viewer.data().set_colors(C);
        }
    };

    viewer.callback_key_down = [&](igl::opengl::glfw::Viewer& v, const unsigned char key, int mod) {
        if (key == 'N') {
            maxIteration += 1;
            options.maxIterationNum = maxIteration;

            cluster.Run(options);
            set_color_to_mesh();
        }
        return false;
    };

    bool is_converaged = false;
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer& v) {
        if (cluster.UpdateCluster()) {
            set_color_to_mesh();
            v.data().set_colors(C);
        } else {
            if (!is_converaged) {
                const auto& uv_bounds = cluster.GetChartUVBounds();
                for (const auto& uv_bound : uv_bounds) {
                    const auto size = uv_bound.size();
                    spdlog::info("uv bounds - [{}, {}]", size.x(), size.y());
                }

                is_converaged = true;
                spdlog::info("succeed to converaged, chart count: {}", cluster.GetClusterCount());
            }
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
    viewer.data().shininess = 100.0f;
    viewer.launch();

    return 0;
}