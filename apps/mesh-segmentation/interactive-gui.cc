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
#include "interactive-gui.h"
#include "chart-packer.h"

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



void VisualizeMesh(const Mesh& mesh, Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C)
{
    V = Eigen::MatrixXd::Zero(static_cast<int>(mesh.n_vertices()), 3);
    F = Eigen::MatrixXi::Zero(static_cast<int>(mesh.n_faces()), 3);
    C = Eigen::MatrixXd::Zero(static_cast<int>(mesh.n_faces()), 3);

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
        const auto& color = mesh.color(faceHandle);
        const auto mesh_color = Eigen::Vector3d(color[0] / 255.0f, color[1] / 255.0f, color[2] / 255.0f);
        C.row(faceHandle.idx()) = mesh_color;
    }
}

void VisualizeMesh(const Mesh& mesh, igl::opengl::glfw::Viewer& viewer)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd C;

    VisualizeMesh(mesh, V, F, C);

    viewer.data().clear();
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
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
    } else {
        spdlog::info("succeed to find input mesh file, file name: {0}, file path: {1}", file_name, path);
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
    mesh.request_vertex_texcoords2D();

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(static_cast<int>(mesh.n_vertices()), 3);
    Eigen::MatrixXi F = Eigen::MatrixXi::Zero(static_cast<int>(mesh.n_faces()), 3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(static_cast<int>(mesh.n_faces()), 3);

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

    ChartPacker::Options chart_options;
    chart_options.enable_space_locality = true;
    chart_options.xatlas_options.bruteForce = true;
    chart_options.xatlas_options.resolution = 1024;
    chart_options.xatlas_options.texelsPerUnit = 128;

    IterativeCluster::Options options;
    options.maxChartArea = options_data["max-chart-area"].get<float>();
    options.minClusterCnt = options.maxChartArea > 0 ? static_cast<int>(std::ceil(area / options.maxChartArea)) : 1;
    options.maxIterationNum = options_data["max-iteration-num"].get<int>();
    options.normalWeight = options_data["normal-weight"].get<float>();
    options.enableUVbounds = options_data["enable-uv-bounds"].get<bool>();

    auto max_uv_size = glm::vec2(8.0f, 8.0f);
    max_uv_size.x = options_data["max-uv-size-x"].get<float>();
    max_uv_size.y = options_data["max-uv-size-y"].get<float>();
    options.maxUVSize = max_uv_size;

    options.maxUVSize.x = std::floor(static_cast<float>(chart_options.xatlas_options.resolution) / chart_options.xatlas_options.texelsPerUnit);
    options.maxUVSize.y = std::floor(static_cast<float>(chart_options.xatlas_options.resolution) / chart_options.xatlas_options.texelsPerUnit);
    spdlog::info("max uv size: [{0}, {1}]", options.maxUVSize.x, options.maxUVSize.y);

    chart_options.xatlas_options.texelsPerUnit = 64;

    cluster.Init(options);

    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);

    int totalClusterCnt = options.minClusterCnt + 10; // static_cast<int>(cluster.GetClusterCount());
    //ColorSetGenerator colorSetGenerator(totalClusterCnt);
    //auto colors = colorSetGenerator.GetColorSet();

    auto set_color_to_mesh = [&]() {
        auto count = cluster.GetClusterCount();
        auto colorGenerator = ColorSetGenerator(static_cast<int>(count));
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

    bool visualize_atlas_mesh = false;
    int visualized_atlas_mesh_index = 0;
    std::vector<Mesh> atlas_meshes;

    bool visualize_merged_atlas_mesh = false;
    Mesh merged_mesh;
    Eigen::MatrixXd atlas_mesh_V, atlas_mesh_C;
    Eigen::MatrixXi atlas_mesh_F;

    int maxIteration = options.maxIterationNum;
    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        if (ImGui::InputInt("max iteration", &maxIteration)) {
            options.maxIterationNum = maxIteration;
            cluster.Run(options);
            set_color_to_mesh();
        }

        if (ImGui::Checkbox("visualize merged atlas mesh", &visualize_merged_atlas_mesh)) {
            if (visualize_merged_atlas_mesh) {
                viewer.data().clear();
                viewer.data().set_mesh(atlas_mesh_V, atlas_mesh_F);
                viewer.data().set_colors(atlas_mesh_C);
            } else {
                viewer.data().clear();
                viewer.data().set_mesh(V, F);
                viewer.data().set_colors(C);
            }
        }

        if (!atlas_meshes.empty()) {
            ImGui::Checkbox("visualize atlas mesh", &visualize_atlas_mesh);
            if (visualize_atlas_mesh) {
                if (ImGui::SliderInt("atlas mesh index", &visualized_atlas_mesh_index, 0, static_cast<int>(atlas_meshes.size()) - 1)) {
                    VisualizeMesh(atlas_meshes[visualized_atlas_mesh_index], viewer);
                }
            }
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

    bool is_converged = false;
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer& v) {
        if (cluster.UpdateCluster()) {
            set_color_to_mesh();
            v.data().set_colors(C);
        } else {
            if (!is_converged) {
                is_converged = true;
                spdlog::info("succeed to converged, chart count: {}", cluster.GetClusterCount());

                const std::string output_file_name = fs::path(path).stem().string() + "_out";

                atlas_meshes = GeneratePackedClusterMesh(cluster, chart_options, output_file_name);
                spdlog::info("atlas mesh count: {}", atlas_meshes.size());

                merged_mesh.request_vertex_texcoords2D();
                merged_mesh.request_face_colors();

                const ColorSetGenerator color_set_generator(atlas_meshes.size() + 10);
                const auto& color_set = color_set_generator.GetColorSet();

                for (int i = 0; i < atlas_meshes.size(); ++ i) {
                    auto& mesh = atlas_meshes[i];
                    mesh.request_face_colors();
                    const auto& color = color_set[i];
                    for (const auto fh : mesh.faces()) {
                        auto mesh_color = Mesh::Color(color.x() * 255, color.y() * 255, color.z() * 255);
                        mesh.set_color(fh, mesh_color);
                    }
                    merged_mesh = MergeMesh(merged_mesh, mesh);
                }

                VisualizeMesh(merged_mesh, atlas_mesh_V, atlas_mesh_F, atlas_mesh_C);

                if (visualize_merged_atlas_mesh) {
                    viewer.data().clear();
                    viewer.data().set_mesh(V, F);
                    viewer.data().set_colors(C);
                }
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
    viewer.resize(1920 * 1.5, 1080 * 1.5);
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.data().shininess = 100.0f;
    viewer.launch();

    return 0;
}

std::vector<Mesh> GeneratePackedClusterMesh(const IterativeCluster& cluster, const ChartPacker::Options& pack_options,  const std::string& file_name)
{
    const auto& uv_bounds = cluster.GetChartUVBounds();
    for (const auto& uv_bound : uv_bounds) {
        const auto size = uv_bound.size();
        spdlog::info("uv bounds - [{}, {}]", size.x, size.y);
    }

    const auto chart_meshes = cluster.Unwrap();
    return WriteChartMesh(cluster, pack_options, file_name, chart_meshes);
}


std::vector<Mesh> WriteChartMesh(const IterativeCluster& cluster, const ChartPacker::Options& pack_options, const std::string& file_name, const std::vector<Mesh>& chart_meshes)
{
    const OpenMesh::IO::Options option = OpenMesh::IO::Options::VertexTexCoord;

    for (int i = 0; i < chart_meshes.size(); ++ i) {
        const auto& chart_mesh = chart_meshes[i];
        auto file_path = fmt::format("data/result/{0}_{1}.obj", file_name, i);
        //OpenMesh::IO::write_mesh(chart_mesh, file_path, option);
    }

    Mesh merged_mesh;
    merged_mesh.request_vertex_texcoords2D();

    for (const auto& chart_mesh : chart_meshes) {
        MergeMesh(merged_mesh, chart_mesh);
    }

    const std::string mesh_file_path = fmt::format("data/result/{0}.obj", file_name);
    OpenMesh::IO::write_mesh(merged_mesh, mesh_file_path, option);

    // TODO: // pack options
    //xatlas::PackOptions pack_options;
    //pack_options.bruteForce = true;
    //pack_options.resolution = 1024;
    //pack_options.texelsPerUnit = 500;

    /*
    if (!Pack(merged_mesh, pack_options)) {
        spdlog::error("failed to pack uv chart");
        return;
    } else {
        spdlog::info("succeed to pack uv chart");
    }

    const std::string uv_mesh_file_path = fmt::format("data/result/{0}_pack.obj", file_name);
    OpenMesh::IO::write_mesh(merged_mesh, uv_mesh_file_path, option);
    */

    ChartPacker chart_packer;

    std::vector<Mesh> atlas_meshes;
    if (!chart_packer.Pack(cluster, pack_options)) {
        spdlog::error("failed to pack uv chart");
        return {};
    } else {
        atlas_meshes = chart_packer.GetAtlasMeshes();
        spdlog::info("succeed to pack uv chart");
    }

    for (int i = 0; i < atlas_meshes.size(); ++ i) {
        const auto& atlas_mesh = atlas_meshes[i];
        const auto atlas_mesh_file_path = fmt::format("data/result/{0}_atlas_{1}.obj", file_name, i);

        OpenMesh::IO::write_mesh(atlas_mesh, atlas_mesh_file_path, option);
        spdlog::info("succeed to write atlas mesh: {0}", atlas_mesh_file_path);
    }
    return atlas_meshes;
}

Mesh MergeMesh(Mesh& merged_mesh, const Mesh& input_mesh)
{
    std::unordered_map<int, int> vertex_map;

    for (const auto vh : input_mesh.vertices()) {
        const auto p = input_mesh.point(vh);
        const auto uv = input_mesh.texcoord2D(vh);

        const auto new_vh = merged_mesh.add_vertex(p);
        merged_mesh.set_texcoord2D(new_vh, uv);

        vertex_map[vh.idx()] = new_vh.idx();
    }

    for (const auto fh : input_mesh.faces()) {
        std::vector<Mesh::VertexHandle> face_handles;
        for (const auto vh : input_mesh.fv_range(fh)) {
            const auto new_vh_idx = vertex_map[vh.idx()];
            auto new_vh = merged_mesh.vertex_handle(new_vh_idx);
            face_handles.push_back(new_vh);
        }
        auto new_fh = merged_mesh.add_face(face_handles);

        if (input_mesh.has_face_colors()) {
            merged_mesh.set_color(new_fh, input_mesh.color(fh));
        }
    }
    return merged_mesh;
}
