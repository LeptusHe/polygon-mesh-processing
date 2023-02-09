#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_corner_normals.h>
#include <igl/opengl/glfw/Viewer.h>
#include <fmt/format.h>
#include "property/face-normal.h"
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include "mds.h"

Eigen::MatrixXd V;
Eigen::MatrixXi F;
Eigen::MatrixXd N_vertices;
Eigen::MatrixXd N_faces;
Eigen::MatrixXd N_corners;

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
    std::cout << D << std::endl;

    auto uv = MDS.Compute(D, 2).transpose();
    std::cout << fmt::format("uv: row={0}, col={1}", uv.rows(), uv.cols()) << std::endl;
    std::cout << uv << std::endl;

    D = Eigen::MatrixXd::Zero(V.rows(), V.rows());
    for (int i = 0; i < V.rows(); i++) {
        for (int j = 0; j < V.rows(); j++) {
            D(i, j) = (uv.row(i) - uv.row(j)).norm();
        }
    }
    //std::cout << "uv matrix: " << std::endl;
    //std::cout << D << std::endl;

    for (int i = 0; i < V.rows(); ++ i) {
        V.row(i) = Eigen::Vector3d{uv(i, 0), uv(i, 1), 0};
    }

    polyscope::init();
    polyscope::registerSurfaceMesh("mesh", V, F);
    polyscope::show();

    /*
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(uv, F);
    viewer.callback_key_down = &key_down;
    viewer.data().set_normals(N_faces);

    viewer.launch();
     */

    return 0;
}