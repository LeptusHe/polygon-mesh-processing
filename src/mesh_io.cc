#include <fmt/format.h>
#include "mesh_io.h"

namespace meshlib {

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

bool LoadMesh(const std::string& filePath, Mesh& mesh, OpenMesh::IO::Options& opt)
{
    if (!OpenMesh::IO::read_mesh(mesh, filePath, opt)) {
        std::cerr << "failed to load mesh from " << filePath << std::endl;
        return false;
    } else {
        std::cout << fmt::format("succeed to load mesh from [{0}]", filePath) << std::endl;
        PrintMeshInfo(mesh);
        return true;
    }
}

} // namespace meshlib