#include <iostream>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "pmp/clip/uv-clipper.h"

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
    mesh.request_vertex_texcoords2D();

    OpenMesh::IO::Options opt;
    opt = opt | OpenMesh::IO::Options::VertexTexCoord;
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

    if (!mesh.has_vertex_texcoords2D()) {
        spdlog::info("failed to load mesh with tex coord, exit!");
        return -1;
    }

    meshlib::UVClipper clipper;
    clipper.process(mesh);

    const auto clipped_mesh = clipper.get_clipped_mesh();
    PrintMeshInfo(clipped_mesh, "after clip");

    const auto output_dir = "data/result/";
    std::filesystem::create_directories(output_dir);
    try {
        std::filesystem::path input_path(path);
        const auto file_name = input_path.stem().string();
        const auto output_path = output_dir + file_name + "_result.obj";

        OpenMesh::IO::Options write_opt = OpenMesh::IO::Options::VertexTexCoord;
        if (!OpenMesh::IO::write_mesh(clipped_mesh, output_path, write_opt)) {
            std::cerr << fmt::format("failed to write mesh to [{0}]", path) << std::endl;
            return 1;
        } else {
            std::cout << fmt::format("succeed to write mesh to [{0}]", path) << std::endl;
        }
    } catch (std::exception& e) {
        std::cerr << fmt::format("failed to write mesh because [{0}]", e.what()) << std::endl;
        return 1;
    }

    const auto visualized_mesh = mesh;
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(visualized_mesh.n_vertices(), 3);
    Eigen::MatrixXi F = Eigen::MatrixXi::Zero(visualized_mesh.n_faces(), 3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(visualized_mesh.n_vertices(), 3);

    for (auto vertHandle : visualized_mesh.vertices()) {
        auto vert = visualized_mesh.point(vertHandle);
        auto uv = visualized_mesh.texcoord2D(vertHandle);
        V.row(vertHandle.idx()) = Eigen::Vector3d(uv[0], 0, uv[1]);

        if (!vertHandle.is_manifold()) {
            C.row(vertHandle.idx()) = Eigen::Vector3d(1, 0, 0);
        } else {
            C.row(vertHandle.idx()) = Eigen::Vector3d(0.5, 0.5f, 0.5f);
        }
    }


    for (auto faceHandle : visualized_mesh.faces()) {
        auto fvIter = visualized_mesh.cfv_iter(faceHandle);
        for (int i = 0; i < 3; ++ i) {
            F(faceHandle.idx(), i) = fvIter->idx();
            ++ fvIter;
        }
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch();

    return 0;
}