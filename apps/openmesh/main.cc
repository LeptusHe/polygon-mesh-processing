#include <iostream>
#include <filesystem>
#include <fmt/format.h>
#include <Eigen/Core>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

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
    auto path = argc > 1 ? argv[1] : "data/test.obj";

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

    for (int vid = 0; vid < mesh.n_vertices() - 1; ++ vid) {
        auto vertHandle = mesh.vertex_handle(vid);
        mesh.delete_vertex(vertHandle, false);
    }
    PrintMeshInfo(mesh, "after delete vertex");

    for (int face = 0; face < mesh.n_faces() - 1; ++ face) {
        auto faceHandle = mesh.face_handle(face);
        //mesh.delete_face(faceHandle, false);
    }
    mesh.garbage_collection();
    PrintMeshInfo(mesh, "after garbage collection");

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

    return 0;
}