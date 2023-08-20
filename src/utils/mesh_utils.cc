#include "mesh_utils.h"
#include <Eigen/Eigen>
#include <fmt/format.h>
#include <boost/bimap.hpp>
#include <boost/bimap/set_of.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <CGAL/polygon_mesh_processing/remesh_planar_patches.h>
#include <CGAL/polygon_mesh_processing.h>
#include <spdlog/spdlog.h>

namespace meshlib {

void MeshUtils::ConvertMeshToViewer(const Mesh& mesh, igl::opengl::glfw::Viewer& viewer)
{
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);
    Eigen::MatrixXi F = Eigen::MatrixXi::Zero(mesh.n_faces(), 3);

    for (auto vh : mesh.vertices()) {
        auto vert = mesh.point(vh);
        V.row(vh.idx()) = Eigen::Vector3d(vert[0], vert[1], vert[2]);

        if (mesh.has_vertex_colors()) {
            auto color = mesh.color(vh);
            C.row(vh.idx()) = Eigen::Vector3d(color[0], color[1], color[2]);
        }
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

    if (mesh.has_vertex_colors()) {
        viewer.data().set_colors(C);
    }
}

void MeshUtils::ConvertMeshToViewer(const CMesh& mesh, igl::opengl::glfw::Viewer& viewer)
{
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(mesh.vertices().size(), 3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(mesh.num_vertices(), 3);
    Eigen::MatrixXi F = Eigen::MatrixXi::Zero(mesh.faces().size(), 3);

    auto pair = mesh.property_map<CMesh::Vertex_index, Eigen::Vector3d>("v:colors");

    for (auto vh : mesh.vertices()) {
        auto vert = mesh.point(vh);
        V.row(vh.idx()) = Eigen::Vector3d(vert[0], vert[1], vert[2]);

        if (pair.second) {
            C.row(vh.idx()) = pair.first[vh];
        }

    }

    for (auto fh : mesh.faces()) {
        auto fvIter = mesh.vertices_around_face(mesh.halfedge(fh));

        auto index = 0;
        for (auto v : fvIter) {
            F(fh.idx(), index) = v.idx();
            index += 1;
        }
    }

    viewer.data().clear();
    viewer.data().set_mesh(V, F);
    if (pair.second) {
        viewer.data().set_colors(C);
    }
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

int CheckInvalidEdges(CMesh& clean_mesh)
{
    auto col_map = clean_mesh.add_property_map<CMesh::Vertex_index, Eigen::Vector3d>("v:colors").first;
    for (auto vh : clean_mesh.vertices()) {
        col_map[vh] = Eigen::Vector3d(1, 1, 1);
    }

    std::vector<CMesh::face_index> faces;
    int count = 0;
    float cos_theta = 0.99;
    int border_cnt = 0;
    auto point_map = clean_mesh.points();
    for (auto e : CGAL::edges(clean_mesh)) {
        auto half_edge = clean_mesh.halfedge(e);
        auto sv = clean_mesh.source(half_edge);
        auto dv = clean_mesh.target(half_edge);

        if (clean_mesh.is_border(e)) {
            border_cnt += 1;

            col_map[sv] = Eigen::Vector3d(1, 0, 0);
            col_map[dv] = Eigen::Vector3d(1, 0, 0);

            continue;
        }

        if (!pmp::Planar_segmentation::is_edge_between_coplanar_faces<Kernel>(e, clean_mesh, cos_theta, point_map)) {
            col_map[sv] = Eigen::Vector3d(0, 1, 0);
            col_map[dv] = Eigen::Vector3d(0, 1, 0);

            auto f1 = clean_mesh.face(half_edge);
            auto op_edge = clean_mesh.opposite(half_edge);
            auto f2 = clean_mesh.face(op_edge);

            auto area1 = pmp::face_area(f1, clean_mesh);
            auto area2 = pmp::face_area(f2, clean_mesh);

            auto n1 = pmp::compute_face_normal(f1, clean_mesh);
            auto n2 = pmp::compute_face_normal(f2, clean_mesh);


            Kernel::Vector_3 n(0, 1, 0);
            if (CGAL::scalar_product(n1, n) < 0) {
                faces.push_back(f1);
            }

            if (CGAL::scalar_product(n2, n) < 0) {
                faces.push_back(f2);
            }

            //bool edge = pmp::Planar_segmentation::is_edge_between_coplanar_faces<Kernel>(e, clean_mesh, cos_theta, point_map);

            count += 1;
        }
    }
    pmp::reverse_face_orientations(faces, clean_mesh);

    spdlog::info("border edge: {}", border_cnt);
    spdlog::info("invalid edge: {0}", count);
    return count;
}

} // namespace meshlib