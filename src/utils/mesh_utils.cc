#include "mesh_utils.h"
#include <Eigen/Eigen>
#include <fmt/format.h>

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
    Eigen::MatrixXi F = Eigen::MatrixXi::Zero(mesh.faces().size(), 3);

    for (auto vh : mesh.vertices()) {
        auto vert = mesh.point(vh);
        V.row(vh.idx()) = Eigen::Vector3d(vert[0], vert[1], vert[2]);
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

std::vector<CMesh::Point> MeshUtils::GetSortedPoints(const std::vector<CMesh::Point>& data)
{
    auto points = data;
    std::sort(std::begin(points), std::end(points), [=](const CMesh::Point& lhs, const CMesh::Point& rhs) {
        return std::tie(lhs[0], lhs[1], lhs[2]) < std::tie(rhs[0], rhs[1], rhs[2]);
    });
    return points;
}

std::vector<CMesh::Point> MeshUtils::GetSortedPoints(const CMesh& mesh)
{
    std::vector<CMesh::Point> points;
    for (auto vertex : mesh.vertices()) {
        auto p = mesh.point(vertex);
        points.push_back(p);
    }

    return GetSortedPoints(points);
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