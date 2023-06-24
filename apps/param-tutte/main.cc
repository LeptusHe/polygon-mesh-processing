#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <igl/boundary_loop.h>
#include <igl/opengl/glfw/Viewer.h>
#include <fmt/format.h>
#include <unordered_set>
#include <Eigen/SparseQR>
#include <exception>
#include <filesystem>
#include <igl/harmonic.h>
#include <igl/adjacency_list.h>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include "utils/mesh_io.h"

Eigen::MatrixXd V;
Eigen::MatrixXi F;
Eigen::MatrixXd U;

constexpr double pi = 3.14159265358979323846;

namespace fs = std::filesystem;

std::vector<std::string> FindModelFilesInDir(const std::string& dir)
{
    std::vector<std::string> filepaths;

    for (const auto& entry : fs::directory_iterator(dir)) {
        const auto& filepath = entry.path();

        filepaths.push_back(filepath.string());
        std::cout << fmt::format("find file in dir: {}\n", filepath.string());
    }
    return filepaths;
}


Eigen::Vector2d ConvertToUV(double len)
{
    const double segment = 1.0 / 4.0;

    int x = static_cast<int>(std::floor(len / segment));
    double y = std::fmod(len, segment) / segment;

    switch (x) {
        case 0: return {0, y};
        case 1: return {y, 1};
        case 2: return {1, 1 - y};
        case 3: return {1 - y, 0};
        default: {
            auto msg = fmt::format("invalid coordinate: {}, {}, len={}", x, y, len);
            throw new std::exception(msg.c_str());
        }
    }
}

Eigen::Vector2d ConvertToCircleUV(double s)
{
    double theta = s * (2.0 * pi);
    double u = std::sin(theta);
    double v = std::cos(theta);
    return {u, v};
}

Eigen::MatrixXd FixBoundary(const Mesh& mesh, std::vector<OpenMesh::VertexHandle>& boundaryVertices)
{
    int boundaryVertCnt = 0;
    for (auto vh: mesh.vertices()) {
        if (!vh.is_boundary())
            continue;

        boundaryVertCnt += 1;
    }

    OpenMesh::HalfedgeHandle start;
    for (auto eh: mesh.halfedges()) {
        if (eh.is_boundary()) {
            start = eh;
        }
    }

    auto cur = start;
    while (true) {
        auto vh = mesh.to_vertex_handle(cur);
        boundaryVertices.push_back(vh);

        auto next = mesh.next_halfedge_handle(cur);
        if (next == start)
            break;

        cur = next;
    }

    Eigen::MatrixXd boundaryUV(boundaryVertices.size(), 2);
    for (int i = 0; i < boundaryVertices.size(); ++i) {
        double val = static_cast<double>(i) / static_cast<double>(boundaryVertices.size());
        auto uv = ConvertToCircleUV(val);
        boundaryUV.row(i) = uv;
    }
    return boundaryUV;
}


Eigen::MatrixXd FixBoundary(const Eigen::MatrixXd& V, const std::vector<int>& boundaryVertList)
{
    Eigen::MatrixXd boundaryUV;
    boundaryUV.resize(boundaryVertList.size(), 2);

    for (int i = 0; i < boundaryVertList.size(); ++ i) {
        double val = static_cast<double>(i) / static_cast<double>(boundaryVertList.size());
        //auto uv = ConvertToUV(val);
        auto uv = ConvertToCircleUV(val);
        //std::cout << fmt::format("({}, {})", uv.x(), uv.y()) << std::endl;
        boundaryUV.row(i) = uv;
    }
    return boundaryUV;
}

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
    switch(key)
    {
        case '1':
            viewer.data().set_vertices(V);
            viewer.core().align_camera_center(V, F);
            return true;
        case '2':
            viewer.data().set_vertices(U);
            viewer.core().align_camera_center(U, F);
            return true;
        default: break;
    }
    return false;
}

using TexMat = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>;

std::vector<TexMat> GenerateGridTex(Eigen::Vector3d colorA, Eigen::Vector3d colorB)
{
    unsigned size = 128;
    unsigned halfSize = size / 2;

    std::vector<TexMat> texList {
        TexMat::Zero(size, size),
        TexMat::Zero(size, size),
        TexMat::Zero(size, size),
    };

    for (int channel = 0; channel < 3; ++ channel) {
        auto& tex = texList[channel];

        for (unsigned i = 0; i < size; ++i) {
            for (unsigned j = 0; j < size; ++j) {
                tex(i, j) = std::clamp(static_cast<int>(colorA[channel] * 255), 0, 255);
                if ((i < halfSize && j < halfSize) || (i >= halfSize && j >= halfSize))
                    tex(i, j) = std::clamp(static_cast<int>(colorB[channel] * 255), 0, 255);
            }
        }
    }

    return texList;

    //texture_G = texture_R;
    //texture_B = texture_R;
    //texture_A = Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>::Constant(texture_R.rows(),texture_R.cols(),255);

    //viewer.data().set_texture()
}

float CalculateTanTheta(OpenMesh::Vec3f& e0, OpenMesh::Vec3f e1)
{
    auto cosTheta = e0.dot(e1) / (e0.norm() * e1.norm());
    auto theta = std::acos(cosTheta);
    return std::tan(theta / 2.0f);
}

float CalculateWeight(const Mesh& mesh, OpenMesh::HalfedgeHandle prev, OpenMesh::HalfedgeHandle cur)
{
    auto v0 = mesh.from_vertex_handle(prev);
    auto v1 = mesh.to_vertex_handle(prev);
    auto v2 = mesh.to_vertex_handle(cur);

    auto p0 = mesh.point(v0);
    auto p1 = mesh.point(v1);
    auto p2 = mesh.point(v2);

    auto e0 = p0 - p1;
    auto e1 = p2 - p1;
    auto tanTheta0 = CalculateTanTheta(e0, e1);
    return tanTheta0;
}

void CalculateMeanValueWeight(Mesh& mesh, OpenMesh::HProp<float>& weightProp, const std::unordered_set<OpenMesh::VertexHandle>& boundaryVertices)
{
    for (auto vh : mesh.vertices()) {
        if (boundaryVertices.find(vh) != std::end(boundaryVertices))
            continue;

        //if (vh.is_boundary())
        //    continue;

        auto sum = 0.0f;
        for (auto eh : mesh.voh_range(vh)) {
            auto prev = eh.prev();
            auto tanTheta0 = CalculateWeight(mesh, prev, eh);

            prev = eh.opp();
            auto cur = prev.next();
            auto tanTheta1 = CalculateWeight(mesh, prev, cur);

            auto p0 = mesh.point(mesh.from_vertex_handle(eh));
            auto p1 = mesh.point(mesh.to_vertex_handle(eh));
            auto weight = (tanTheta0 + tanTheta1) / (p1 - p0).length();

            weightProp[eh] = weight;
            sum += weight;
        }

        for (auto eh : mesh.voh_range(vh)) {
            auto weight = weightProp[eh];
            weightProp[eh] = weight / sum;
        }
    }
}

std::unordered_set<OpenMesh::VertexHandle> GetBoundaryVertexSet(const std::vector<OpenMesh::VertexHandle>& boundaryVertices)
{
    std::unordered_set<OpenMesh::VertexHandle> set;
    for (const auto vertex : boundaryVertices) {
        set.insert(vertex);
    }
    return set;
}


Eigen::SparseMatrix<double> Solve(Mesh& mesh, const std::vector<OpenMesh::VertexHandle>& boundaryVertices, Eigen::MatrixXd boundaryUV)
{
    auto weightProp = OpenMesh::HProp<float>(mesh, "weight");

    auto boundaryVertexSet = GetBoundaryVertexSet(boundaryVertices);
    CalculateMeanValueWeight(mesh, weightProp, boundaryVertexSet);

    Eigen::SparseMatrix<double> A = Eigen::SparseMatrix<double>(V.rows(), V.rows());
    A.setZero();
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(V.rows(), 2);

    for (int i = 0; i < boundaryVertices.size(); ++ i) {
        auto vh = boundaryVertices[i];
        auto idx = vh.idx();

        auto uv = boundaryUV.row(i);
        b.row(idx) = uv;
    }

    for (auto vh : mesh.vertices()) {
        if (boundaryVertexSet.find(vh) != std::end(boundaryVertexSet)) {
            A.insert(vh.idx(), vh.idx()) = 1;
        } else {
            A.insert(vh.idx(), vh.idx()) = 0;

            for (auto adjacencyVertex : mesh.vv_range(vh)) {
                auto eh = mesh.find_halfedge(vh, adjacencyVertex);
                float weight = weightProp[eh];
                //weight = 1.0f;

                A.insert(vh.idx(), adjacencyVertex.idx()) = weight;
                A.coeffRef(vh.idx(), vh.idx()) -= weight;
            }
        }
    }
    return A;
}


int main(int argc, char *argv[])
{
    auto path = argc > 1 ? argv[1] : "data/beetle.obj";

    Mesh mesh;
    OpenMesh::IO::Options opt;

    if (!meshlib::LoadMesh(path, mesh, opt))
        return -1;
    mesh.request_vertex_texcoords2D();


    if (!igl::read_triangle_mesh(path, V, F)) {
        std::cout << fmt::format("failed to load mesh: [{0}]", path);
        return -1;
    } else {
        std::cout << fmt::format("vertex count: {}\n", V.rows());
        std::cout << fmt::format("face count: {}\n", F.rows());
    }

    //std::vector<int> boundaryVertIndexList;
    //igl::boundary_loop(F, boundaryVertIndexList);

    std::vector<OpenMesh::VertexHandle> boundaryVertices;
    //auto boundaryUV = FixBoundary(V, boundaryVertIndexList);
    auto boundaryUV = FixBoundary(mesh, boundaryVertices);

    std::vector<int> boundaryVertIndexList;
    for (const auto& index : boundaryVertices) {
        boundaryVertIndexList.push_back(index.idx());
    }

    Eigen::MatrixXd colorData = Eigen::MatrixXd::Zero(V.rows(), 3);
    int cnt = 0;
    int segCnt = boundaryVertIndexList.size() / 3;

    for (const auto index : boundaryVertIndexList) {
        double val = static_cast<double>(cnt) / boundaryVertIndexList.size();
        cnt += 1;
        Eigen::Vector3d col;

        if (cnt <= segCnt) {
            col = Eigen::Vector3d(1, 0, 0);
        } else if (cnt < 2 * segCnt) {
            col = Eigen::Vector3d(0, 1, 0);
        } else {
            col = Eigen::Vector3d(0, 0, 1);
        }

        colorData.row(index) = col; // Eigen::Vector3d(val, 0, 0);
    }

    for (int i = 0; i < boundaryVertIndexList.size(); ++ i) {
        auto vertIndex = boundaryVertIndexList[i];

        auto uv = boundaryUV.row(i);
        colorData.row(vertIndex) = Eigen::Vector3d{uv.x(), uv.y(), 0.0};
    }

    std::unordered_set<int> boundaryVertexSet;
    for (const auto vertIndex : boundaryVertIndexList) {
        boundaryVertexSet.insert(vertIndex);
    }

    Eigen::SparseMatrix<double> A = Eigen::SparseMatrix<double>(V.rows(), V.rows());
    A.setZero();
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(V.rows(), 2);

    for (int i = 0; i < boundaryVertIndexList.size(); ++ i) {
        auto vertIndex = boundaryVertIndexList[i];

        auto uv = boundaryUV.row(i);
        b.row(vertIndex) = uv;
    }

    for (int f = 0; f < F.rows(); ++ f) {
        const auto face = F.row(f);

        for (int i = 0; i < 3; ++ i) {
            int l = face[(i + 0) % 3];
            int r = face[(i + 1) % 3];

            bool isBoundaryVert = boundaryVertexSet.find(l) != std::end(boundaryVertexSet);
            if (isBoundaryVert) {
                //A.coeffRef(l, l) = 1;
            } else {
                //A.coeffRef(l, r) = 1;
                //A.coeffRef(l, l) -= 1;
            }
        }
    }

    std::vector<std::vector<int>> vertexAdjacencyList;
    igl::adjacency_list(F, vertexAdjacencyList);
    for (int i = 0; i < V.rows(); ++ i) {
        const auto& adjacencyList = vertexAdjacencyList[i];

        bool isBoundaryVert = boundaryVertexSet.find(i) != std::end(boundaryVertexSet);
        if (isBoundaryVert) {
            A.insert(i, i) = 1;
            continue;
        }

        A.insert(i, i) = 0;
        for (int adjacencyIndex : adjacencyList) {
            A.insert(i, adjacencyIndex) = 1;
            A.coeffRef(i, i) -= 1;

            //std::cout << A.coeff(i, adjacencyIndex) << std::endl;
            //std::cout << A.coeff(i, adjacencyIndex) << std::endl;
        }
    }

    A = Solve(mesh, boundaryVertices, boundaryUV);

    for (int i = 0; i < A.rows(); ++ i) {
        for (int j = 0; j < A.cols(); ++ j) {
            if (A.coeff(i, j) != 0) {
                //std::cout << A.coeffRef(i, j) << ", ";
            }
        }
        //std::cout << std::endl;
    }
    //std::cout << A << std::endl;

#if ENABLE_DEBUG
    std::cout << "A: " << std::endl;
    std::cout << A << std::endl;

    std::cout << "b: " << std::endl;
    std::cout << b << std::endl;
#endif

    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;

    solver.compute(A);
    if (solver.info() != Eigen::Success) {
        std::cout << "failed to init solver" << std::endl;
        return -1;
    }

    U = solver.solve(b);
    if (solver.info() != Eigen::Success) {
        std::cout << "failed to solve" << std::endl;
        return -1;
    }

#if ENABLE_DEBUG
    std::cout << "solve: " << std::endl;
    std::cout << U << std::endl;
#endif

    Eigen::VectorXi bnd;
    igl::boundary_loop(F, bnd);
    //igl::harmonic(V, F, bnd, boundaryUV, 2, U);

    Eigen::MatrixXd scaleUV = 10 * U;
    auto texList = GenerateGridTex(Eigen::Vector3d{1, 1, 1}, Eigen::Vector3d{0.5, 0.0, 0.0});

    igl::opengl::glfw::Viewer viewer;
    auto modelId = viewer.data_list[0].id;
    viewer.data(modelId).set_mesh(V, F);
    viewer.data(modelId).set_colors(U);
    viewer.data(modelId).set_uv(scaleUV);
    viewer.data(modelId).show_texture = true;
    viewer.data(modelId).uniform_colors(Eigen::Vector3d {1, 1, 1}, Eigen::Vector3d {1, 1, 1}, Eigen::Vector3d{0, 0, 0});
    viewer.data(modelId).set_texture(texList[0], texList[1], texList[2]);

    int index = viewer.append_mesh();
    viewer.data(index).set_mesh(U, F);
    viewer.data(index).set_colors(U);
    viewer.data(index).set_uv(scaleUV);
    viewer.data(index).show_texture = true;
    viewer.data(index).uniform_colors(Eigen::Vector3d {1, 1, 1}, Eigen::Vector3d {1, 1, 1}, Eigen::Vector3d{0, 0, 0});
    viewer.data(index).set_texture(texList[0], texList[1], texList[2]);

    int w = 1080, h = 960;
    int left_view, right_view;

    viewer.callback_init = [&](igl::opengl::glfw::Viewer &v) -> bool {
        v.core().viewport = Eigen::Vector4f(0, 0, w / 2, h);
        left_view = v.core_list[0].id;

        auto viewport = Eigen::Vector4f(w / 2, 0, w - (w / 2), h);
        right_view = viewer.append_core(viewport);

        v.data(modelId).set_visible(true, left_view);
        v.data(modelId).set_visible(false, right_view);

        v.data(index).set_visible(false, left_view);
        v.data(index).set_visible(true, right_view);
        return false;
    };


    viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h) {
        v.core(left_view).viewport = Eigen::Vector4f(0, 0, w / 2, h);
        v.core(right_view).viewport = Eigen::Vector4f(w / 2, 0, w - (w / 2), h);
        return true;
    };

    viewer.callback_key_down = &key_down;

    viewer.launch(false, false, "viewer", w, h);

    return 0;
}