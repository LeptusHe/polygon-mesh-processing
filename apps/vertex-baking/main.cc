#include <iostream>
#include <filesystem>
#include <fmt/format.h>
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/png/readPNG.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "texture/texture.h"
#include "point-sampling-vertex-baker.h"
#include "least-squares-vertex-baker.h"

using namespace meshlib;

enum class BakingMethod {
    PointSampling,
    LeastSquares
};

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
    auto path = argc > 1 ? argv[1] : "data/quad_100_100.obj";

    std::string tex_file_path = "./data/baking/PGD_WildBossA_01_01_D.png";
    path = "./data/baking/PGD_WildBossA.obj";
    //path = "./data/baking/plane_test_30_30.obj";
    //path = "./data/baking/plane_test_20_20.obj";
    path = "./data/baking/plane_test_10_10.obj";
    //path = "./data/baking/random_plane_10.0_8.0_100.obj";

    Texture tex;
    if (!tex.Load(tex_file_path)) {
        return -1;
    }

    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
    if (!igl::png::readPNG(tex_file_path, R, G, B, A)) {
        spdlog::error("failed to read texture from path [{}]", tex_file_path);
    }

    Mesh mesh;
    mesh.request_vertex_texcoords2D();
    mesh.request_vertex_colors();

    OpenMesh::IO::Options opt = OpenMesh::IO::Options::VertexTexCoord;
    if (!OpenMesh::IO::read_mesh(mesh, path, opt)) {
        std::cerr << "failed to load mesh from " << path << std::endl;
        return 1;
    } else {
        std::cout << fmt::format("succeed to load mesh from [{0}]", path) << std::endl;
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
    int current_method = static_cast<int>(BakingMethod::PointSampling);
    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        bool resolve_vertex_color = false;
        if (ImGui::Checkbox("debug integral method", &debug_integral_method)) {
            resolve_vertex_color = true;
        }

        ImGui::Separator();

        const char* items[] = {"Point Sampling", "Least Squares"};
        if (ImGui::Combo("method", &current_method, items, IM_ARRAYSIZE(items)) || resolve_vertex_color) {
            auto method = static_cast<BakingMethod>(current_method);
            std::unique_ptr<VertexBaker> vertex_baker;
            switch (method) {
                case BakingMethod::PointSampling: {
                    vertex_baker = std::make_unique<PointSamplingVertexBaker>(mesh, tex);
                    break;
                }
                case BakingMethod::LeastSquares: {
                    LeastSquaresVertexBaker::Options options;
                    options.debug_integral_method = debug_integral_method;
                    vertex_baker = std::make_unique<LeastSquaresVertexBaker>(mesh, tex, options);
                    break;
                }
                default: {
                    spdlog::error("invalid baking method");
                }
            }
            vertex_baker->Solve();
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