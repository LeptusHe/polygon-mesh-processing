#include <iostream>
#include <filesystem>
#include <fmt/format.h>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "vertex-baker.h"

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

    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();

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
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch();

    return 0;
}