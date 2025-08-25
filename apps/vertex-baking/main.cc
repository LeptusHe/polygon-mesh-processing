#include <iostream>
#include <filesystem>
#include <fmt/format.h>
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/png/readPNG.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "error_analyzer.h"
#include "texture/texture.h"
#include "point-sampling-vertex-baker.h"
#include "least-squares-vertex-baker.h"

using namespace meshlib;

enum class OptimizationMethod {
    PointSampling,
    LeastSquares
};

enum class SamplingMethod {
    Random,
    Uniform
};

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;

struct Options {
    std::string input_mesh_path;
    std::string output_mesh_path;
    std::string tex_path;
    int sample_num;
    OptimizationMethod optimization_method;
    SamplingMethod sampling_method;
    bool enable_edge_regularization;
    float regularization_factor;
};

Options ParseConfigFromJson(const std::string& config_path) {
    Options config;
    
    try {
        std::ifstream file(config_path);
        if (!file.is_open()) {
            throw std::runtime_error(fmt::format("Cannot open config file: {}", config_path));
        }
        
        nlohmann::json j;
        file >> j;
        
        config.input_mesh_path = j.value("input mesh path", "");
        config.output_mesh_path = j.value("output mesh path", "");
        config.tex_path = j.value("tex path", "");
        config.sample_num = j.value("sample num", 512);

        const auto opt_method = j.value("optimization method", "least-squares");
        config.optimization_method = opt_method == "least-squares" ? OptimizationMethod::LeastSquares : OptimizationMethod::PointSampling;

        const auto sampling_method = j.value("sampling method", "uniform");
        config.sampling_method = sampling_method == "random" ? SamplingMethod::Random : SamplingMethod::Uniform;
        
        if (j.contains("least squares options")) {
            auto ls_options = j["least squares options"];
            config.enable_edge_regularization = ls_options.value("enable edge regularization", true);
            config.regularization_factor = ls_options.value("regularization factor", 0.1f);
        } else {
            config.enable_edge_regularization = true;
            config.regularization_factor = 0.1f;
        }
        
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config file: {}", e.what());
        throw;
    }
    
    return config;
}


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
    std::string config_path = argc > 1 ? argv[1] : "apps/vertex-baking/docs/arguments.json";
    
    Options config;
    try {
        config = ParseConfigFromJson(config_path);
        spdlog::info("Successfully loaded config from: {}", config_path);
    } catch (const std::exception& e) {
        spdlog::error("failed to load config from: {}", config_path);
        throw;
    }

    Texture tex;
    if (!tex.Load(config.tex_path)) {
        return -1;
    }

    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
    if (!igl::png::readPNG(config.tex_path, R, G, B, A)) {
        spdlog::error("failed to read texture from path [{}]", config.tex_path);
    }

    Mesh mesh;
    mesh.request_vertex_texcoords2D();
    mesh.request_vertex_colors();

    OpenMesh::IO::Options opt = OpenMesh::IO::Options::VertexTexCoord;
    if (!OpenMesh::IO::read_mesh(mesh, config.input_mesh_path, opt)) {
        std::cerr << "failed to load mesh from " << config.input_mesh_path << std::endl;
        return 1;
    } else {
        std::cout << fmt::format("succeed to load mesh from [{0}]", config.input_mesh_path) << std::endl;
        PrintMeshInfo(mesh);
    }

    if (!mesh.has_vertex_texcoords2D()) {
        spdlog::error("loaded mesh does not have tex coordinate");
        return -1;
    }

    if (!mesh.has_vertex_colors()) {
        spdlog::error("loaded mesh does not have vertex colors");
        return -1;
    }

    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();

    PointSamplingVertexBaker vertex_baker(mesh, tex);
    vertex_baker.Solve();

    std::filesystem::create_directories(std::filesystem::path(config.output_mesh_path).parent_path());
    try {

        if (!OpenMesh::IO::write_mesh(mesh, config.output_mesh_path)) {
            std::cerr << fmt::format("failed to write mesh to [{0}]", config.output_mesh_path) << std::endl;
            return 1;
        } else {
            std::cout << fmt::format("succeed to write mesh to [{0}]", config.output_mesh_path) << std::endl;
        }
    } catch (std::exception& e) {
        std::cerr << fmt::format("failed to write mesh because [{0}]", e.what()) << std::endl;
        return 1;
    }

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);
    Eigen::MatrixXi F = Eigen::MatrixXi::Zero(mesh.n_faces(), 3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);

    for (auto vertHandle : mesh.vertices()) {
        auto vert = mesh.point(vertHandle);
        //std::cout << fmt::format("vert: [{0}, {1}, {2}]", vert[0], vert[1], vert[2]) << std::endl;
        V.row(vertHandle.idx()) = Eigen::Vector3d(vert[0], vert[1], vert[2]);

        if (!vertHandle.is_manifold()) {
            C.row(vertHandle.idx()) = Eigen::Vector3d(1, 0, 0);
        } else {
            C.row(vertHandle.idx()) = Eigen::Vector3d(0.5, 0.5f, 0.5f);
        }
    }

    for (auto faceHandle : mesh.faces()) {
        auto fvIter = mesh.cfv_iter(faceHandle);
        for (int i = 0; i < 3; ++ i) {
            F(faceHandle.idx(), i) = fvIter->idx();
            //std::cout << fmt::format("face: [{0}, {1}, {2}]", faceHandle.idx(), i, fvIter->idx()) << std::endl;
            ++ fvIter;
        }
    }

    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&plugin);
    plugin.widgets.push_back(&menu);

    bool debug_integral_method = false;
    bool enable_random_sample = config.sampling_method == SamplingMethod::Random;
    bool enable_edge_regularization = config.enable_edge_regularization;
    float regularization_factor = config.regularization_factor;
    int sample_num = config.sample_num;
    int current_method = static_cast<int>(config.optimization_method);
    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        bool regenerate_vertex_color = false;
        if (ImGui::Checkbox("debug integral method", &debug_integral_method)) {
            regenerate_vertex_color = true;
        }

        if (ImGui::Checkbox("enable random sample", &enable_random_sample)) {
            regenerate_vertex_color = true;
        }

        if (ImGui::Checkbox("enable edge regularization", &enable_edge_regularization)) {
            regenerate_vertex_color = true;
        }

        if (enable_edge_regularization) {
            if (ImGui::SliderFloat("regularization factor", &regularization_factor, 0.0f, 1.0f)) {
                regenerate_vertex_color = true;
            }
        }

        if (ImGui::SliderInt("sample num", &sample_num, 4, 4 * 1024)) {
            regenerate_vertex_color = true;
        }

        ImGui::Separator();

        const char* items[] = {"Point Sampling", "Least Squares"};
        if (ImGui::Combo("method", &current_method, items, IM_ARRAYSIZE(items)) || regenerate_vertex_color) {
            auto method = static_cast<OptimizationMethod>(current_method);
            std::unique_ptr<VertexBaker> vertex_baker;
            switch (method) {
                case OptimizationMethod::PointSampling: {
                    vertex_baker = std::make_unique<PointSamplingVertexBaker>(mesh, tex);
                    break;
                }
                case OptimizationMethod::LeastSquares: {
                    LeastSquaresVertexBaker::Options options;
                    options.debug_integral_method = debug_integral_method;
                    options.enable_random_sample = enable_random_sample;
                    options.enable_edge_regularization = enable_edge_regularization;
                    options.regularization_factor = regularization_factor;
                    options.sample_num = sample_num;
                    vertex_baker = std::make_unique<LeastSquaresVertexBaker>(mesh, tex, options);
                    break;
                }
                default: {
                    spdlog::error("invalid baking method");
                }
            }
            vertex_baker->Solve();

            ErrorAnalyzer analyzer(mesh, tex, 1024);
            const auto error = analyzer.Analyze();
            spdlog::info("analyze error: {}", error);

            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            viewer.data().set_texture(R, G, B, A);
        }

        ImGui::Spacing();
        if (ImGui::Button("show texture")) {
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            viewer.data().set_texture(R, G, B, A);

            Eigen::MatrixXd C = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);
            for (int i = 0; i < mesh.n_vertices(); ++ i) {
                C.row(i) = Eigen::Vector3d(1.0f, 1.0f, 1.0f);
            }
            viewer.data().set_colors(C);
            viewer.data().show_texture = true;
        }
        ImGui::Spacing();
    };


    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
    viewer.data().set_texture(R, G, B, A);

    viewer.launch();

    return 0;
}