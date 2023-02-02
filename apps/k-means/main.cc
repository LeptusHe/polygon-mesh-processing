#include <iostream>
#include <filesystem>
#include <fmt/format.h>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "kmeans.h"
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
    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);
    Eigen::MatrixXi F = Eigen::MatrixXi::Zero(mesh.n_faces(), 3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);

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

    auto clusterProp = OpenMesh::VProp<int>(mesh, "cluster");
    KMeans kMeans(mesh, clusterProp);

    int clusterCnt = 10;
    kMeans.Run(clusterCnt, 100);

    ColorSetGenerator colorSetGenerator(clusterCnt);
    auto colors = colorSetGenerator.GetColorSet();

    for (auto vertHandle : mesh.vertices()) {
        auto cluster = clusterProp[vertHandle];
        C.row(vertHandle.idx()) = colors[cluster];
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch();

    return 0;
}