#include "MeshUtils.h"
#include <Eigen/Eigen>
#include <fmt/format.h>

namespace meshlib {

void MeshUtils::ConvertMeshToViewer(const Mesh& mesh, igl::opengl::glfw::Viewer& viewer)
{
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);
    Eigen::MatrixXi F = Eigen::MatrixXi::Zero(mesh.n_faces(), 3);

    for (auto vh : mesh.vertices()) {
        auto vert = mesh.point(vh);
        V.row(vh.idx()) = Eigen::Vector3d(vert[0], vert[1], vert[2]);
    }

    for (auto fh : mesh.faces()) {
        auto fvIter = mesh.cfv_iter(fh);
        for (int i = 0; i < 3; ++ i) {
            F(fh.idx(), i) = fvIter->idx();
            ++ fvIter;
        }
    }

    viewer.data().clear();
    viewer.data().set_mesh(V, F);
    viewer.data().show_vertex_labels = true;
}

bool MeshUtils::WriteMesh(const Mesh &mesh, const std::string &path)
{
    try {
        if (!OpenMesh::IO::write_mesh(mesh, path)) {
            std::cerr << fmt::format("failed to write mesh to [{0}]", path) << std::endl;
            return true;
        } else {
            std::cout << fmt::format("succeed to write mesh to [{0}]", path) << std::endl;
            return false;
        }
    } catch (std::exception& e) {
        std::cerr << fmt::format("failed to write mesh because [{0}]", e.what()) << std::endl;
        return false;
    }
    return true;
}

} // namespace meshlib