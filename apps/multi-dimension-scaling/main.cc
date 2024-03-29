#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_corner_normals.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/exact_geodesic.h>
#include <fmt/format.h>
#include "property/face-normal.h"

#define ENABLE_POLYSCOPE_VISUALIZER 1

#if ENABLE_POLYSCOPE_VISUALIZER
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#endif

#include <fmt/ostream.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

#include "parameterization/mds.h"

Eigen::MatrixXd V;
Eigen::MatrixXi F;
Eigen::MatrixXd N_vertices;
Eigen::MatrixXd N_faces;
Eigen::MatrixXd N_corners;

template <> struct fmt::formatter<Eigen::MatrixXd> : fmt::ostream_formatter {};

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
    switch(key)
    {
        case '1':
            viewer.data().set_normals(N_faces);
            return true;
        case '2':
            igl::per_vertex_normals(V, F, igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_UNIFORM, N_vertices);
            viewer.data().set_normals(N_vertices);
            return true;
        case '3':
            igl::per_vertex_normals(V, F, igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA, N_vertices);
            viewer.data().set_normals(N_vertices);
            return true;
        case '4':
            igl::per_vertex_normals(V, F, igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE, N_vertices);
            viewer.data().set_normals(N_vertices);
            return true;
        case '5':
            viewer.data().set_normals(N_corners);
            return true;
        case '6': {
            auto calculator = meshlib::FaceNormalCalculator();
            N_faces = calculator.Calculate(V, F);
            viewer.data().set_normals(N_faces);
            return true;
        }
        default: break;
    }
    return false;
}

int main(int argc, char *argv[])
{
    auto path = argc > 1 ? argv[1] : "data/bunny.off";
    if (!igl::read_triangle_mesh(path, V, F)) {
        std::cout << fmt::format("failed to load mesh: [{0}]", path);
        return -1;
    } else {
        std::cout << fmt::format("load mesh: [{0}]", path) << std::endl;
        std::cout << "vertex count: " << V.rows() << std::endl;
        std::cout << "face count: " << F.rows() << std::endl;
    }

    auto logger = spdlog::basic_logger_mt("basic_logger", "logs/basic.txt");
    auto clogger = spdlog::basic_logger_mt("basic_logger2", "logs/basic2.txt");

    // Compute per-face normals
    igl::per_face_normals(V, F, N_faces);

    // Compute per-vertex normals
    igl::per_vertex_normals(V, F,  N_vertices);

    // Compute per-corner normals, |dihedral angle| > 20 degrees --> crease
    igl::per_corner_normals(V, F, 20, N_corners);


    mds MDS;
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(V.rows(), V.rows());
    for (int i = 0; i < V.rows(); i++) {
        for (int j = 0; j < V.rows(); j++) {
            D(i, j) = (V.row(i) - V.row(j)).norm();
        }
    }


    //std::cout << D << std::endl;
    Eigen::VectorXi VS = Eigen::VectorXi ::Zero(V.rows());
    Eigen::VectorXi VT = Eigen::VectorXi ::Zero(V.rows());

    Eigen::VectorXi FS; // = Eigen::VectorXi ::Zero(F.rows());
    Eigen::VectorXi FT; // = Eigen::VectorXi ::Zero(F.rows());
    Eigen::VectorXd Ds;

    for (int i = 0; i < V.rows(); ++ i) {
        VS(i) = i;
        VT(i) = (i + 10) % V.rows();
    }

    for (int i = 0; i < F.rows(); ++ i) {
        //FS(i) = i;
        //FT(i) = i;
    }

    VS.resize(1);
    for (int i = 0; i < V.rows(); ++ i) {
        // The selected vertex is the source
        VS << i;
        // All vertices are the targets
        VT.setLinSpaced(V.rows(), 0, V.rows() - 1);

        std::cout << "Computing geodesic distance to vertex " << i << "..." << std::endl;

        igl::exact_geodesic(V, F, VS, FS, VT, FT, Ds);

        for (int j = 0; j < V.rows(); ++ j) {
            D(i, j) = Ds(j) * Ds(j);
        }
    }

    igl::exact_geodesic(V, F, VS, FS, VT, FT, Ds);
    std::cout << "ds: row " << Ds.rows() << std::endl;
    std::cout << VS << std::endl;
    std::cout << VT << std::endl;
    std::cout << Ds << std::endl;

    std::cout << "matrix D" << std::endl;
    std::cout << D << std::endl;

    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(V.rows(), 3);
    for (int i = 0; i < V.rows(); ++ i) {
        auto d = D(i);

        auto stripSize = 0.02;
        auto color = std::sin((d / stripSize) * 3.1415926f);
        color = std::abs(color);

        C.row(i) = Eigen::Vector3d(color, color, color);
    }

    logger->info("{0}", D);

    auto uv1 = MDS.Compute(D, 2);
    std::cout << "uv: " << uv1 << std::endl;

    auto uv = uv1.transpose();
    std::cout << "uv: " << uv << std::endl;

    std::cout << fmt::format("uv: row={0}, col={1}", uv.rows(), uv.cols()) << std::endl;
    std::cout << uv << std::endl;

    Eigen::MatrixXd realD = Eigen::MatrixXd::Zero(V.rows(), V.rows());
    for (int i = 0; i < V.rows(); i++) {
        for (int j = 0; j < V.rows(); j++) {
            float v = (uv.row(i) - uv.row(j)).norm();
            realD(i, j) = (uv.row(i) - uv.row(j)).squaredNorm();
        }
    }
    clogger->info("{0}", realD);

    float diff = 0;
    for (int i = 0; i < V.rows(); ++ i) {
        for (int j = 0; j < V.rows(); ++ j) {
            diff += std::abs(D(i, j) - realD(i, j));
        }
    }
    std::cout << "diff: " << diff << std::endl;

    //std::cout << "uv matrix: " << std::endl;
    //std::cout << D << std::endl;

    for (int i = 0; i < V.rows(); ++ i) {
        C.row(i) = Eigen::Vector3d{uv(i, 0), uv(i, 1), 0};
    }

#if ENABLE_POLYSCOPE_VISUALIZER
    polyscope::init();
    polyscope::registerSurfaceMesh("mesh", V, F);
    polyscope::show();

    polyscope::getSurfaceMesh("mesh")->addVertexColorQuantity("vColor", C);
#else
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(C, F);
    viewer.data().set_colors(C);
    viewer.callback_key_down = &key_down;
    viewer.data().set_normals(N_faces);

    viewer.launch();
#endif

    return 0;
}