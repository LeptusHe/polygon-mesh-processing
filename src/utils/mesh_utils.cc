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

        if (!pmp::Planar_segmentation::is_edge_between_coplanar_faces<Kernel>(e, clean_mesh, -cos_theta, point_map)) {
            col_map[sv] = Eigen::Vector3d(0, 1, 0);
            col_map[dv] = Eigen::Vector3d(0, 1, 0);

            auto f1 = clean_mesh.face(half_edge);
            auto op_edge = clean_mesh.opposite(half_edge);
            auto f2 = clean_mesh.face(op_edge);

            auto area1 = pmp::face_area(f1, clean_mesh);
            auto area2 = pmp::face_area(f2, clean_mesh);

            auto n1 = pmp::compute_face_normal(f1, clean_mesh);
            auto n2 = pmp::compute_face_normal(f2, clean_mesh);
            float s = CGAL::scalar_product(n1, n2);


            Kernel::Vector_3 n(0, 1, 0);
            if (CGAL::scalar_product(n1, n) < 0) {
                faces.push_back(f1);
            }

            if (CGAL::scalar_product(n2, n) < 0) {
                faces.push_back(f2);
            }

            bool edge = pmp::Planar_segmentation::is_edge_between_coplanar_faces<Kernel>(e, clean_mesh, cos_theta, point_map);

            count += 1;
        }
    }
    pmp::reverse_face_orientations(faces, clean_mesh);

    spdlog::info("border edge: {}", border_cnt);
    spdlog::info("invalid edge: {0}", count);
    return count;
}

Mesh ConvertToMesh(const CMesh& cmesh)
{
    std::vector<CMesh::Point> points;
    std::vector<std::vector<std::size_t>> polygons;

    pmp::polygon_mesh_to_polygon_soup(cmesh, points, polygons);

    Mesh mesh;
    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();
    mesh.request_face_normals();
    mesh.request_vertex_normals();

    for (int i = 0; i < points.size(); i++) {
        const auto& p = points[i];
        auto vh = mesh.add_vertex(Mesh::Point(p[0], p[1], p[2]));
        mesh.set_normal(vh, Mesh::Normal(0, 0, 1));
    }

    for (int i = 0; i < polygons.size(); ++i) {
        const auto& polygon = polygons[i];

        auto v0 = mesh.vertex_handle(polygon[0]);
        auto v1 = mesh.vertex_handle(polygon[1]);
        auto v2 = mesh.vertex_handle(polygon[2]);

        if (v0.idx() == v1.idx() || v0.idx() == v2.idx() || v1.idx() == v2.idx())
            continue;

        mesh.add_face(v0, v1, v2);
    }
    mesh.update_vertex_normals();

    return mesh;
}

CMesh ConstructSurfaceMesh(const float *vertices, const float* normals, int numVertices, const int *indices, int triangleNum)
{
    assert(vertices != nullptr && indices != nullptr);

    auto mesh = CMesh();

    auto vhList = std::vector<CMesh::Vertex_index>(numVertices);
    for (int i = 0; i < numVertices; ++ i) {
        auto p = CMesh::Point(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
        auto vh = mesh.add_vertex(p);

        vhList[i] = vh;
    }

    int numIndices = 3 * triangleNum;
    for (int i = 0; i < numIndices; i += 3) {
        auto v0Index = indices[i + 0];
        auto v1Index = indices[i + 1];
        auto v2Index = indices[i + 2];

        mesh.add_face(vhList[v0Index],
                       vhList[v1Index],
                       vhList[v2Index]);
    }

    return mesh;
}

void CollectMeshData(const Mesh& mesh, std::vector<float>& vertices, std::vector<int>& indices)
{
    vertices.resize(3 * mesh.n_vertices());
    for (auto vertex : mesh.vertices()) {
        auto idx = vertex.idx();
        auto p = mesh.point(vertex);

        vertices[3 * idx + 0] = p[0];
        vertices[3 * idx + 1] = p[1];
        vertices[3 * idx + 2] = p[2];
    }

    indices.resize(3 * mesh.n_faces());
    for (auto fh : mesh.faces()) {
        auto faceIndex = fh.idx();

        int vIdx = 0;
        for (auto vh : fh.vertices()) {
            indices[3 * faceIndex + vIdx] = vh.idx();
            vIdx += 1;
        }
    }
}

CMesh ConvertOpenMeshToSurfaceMesh(const Mesh& mesh)
{
    std::vector<float> vertices;
    std::vector<int> indices;

    CollectMeshData(mesh, vertices, indices);

    int vertexNum = static_cast<int>(vertices.size()) / 3;
    int faceNum = static_cast<int>(indices.size()) / 3;
    return ConstructSurfaceMesh(vertices.data(), nullptr, vertexNum, indices.data(), faceNum);
}

} // namespace meshlib