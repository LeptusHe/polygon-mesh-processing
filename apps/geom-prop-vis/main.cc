#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_corner_normals.h>
#include <igl/opengl/glfw/Viewer.h>
#include <fmt/format.h>

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
            viewer.data().set_normals(N_vertices);
            return true;
        case '3':
            viewer.data().set_normals(N_corners);
            return true;
        default: break;
    }
    return false;
}

int main(int argc, char *argv[])
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    auto path = argc > 1 ? argv[1] : "data/bunny.off";
    if (!igl::read_triangle_mesh(path, V, F)) {
        std::cout << fmt::format("failed to load mesh: [{0}]", path);
        return -1;
    }

    // Compute per-face normals
    igl::per_face_normals(V, F, N_faces);

    // Compute per-vertex normals
    igl::per_vertex_normals(V, F, N_vertices);

    // Compute per-corner normals, |dihedral angle| > 20 degrees --> crease
    igl::per_corner_normals(V, F, 20, N_corners);

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.callback_key_down = &key_down;
    viewer.data().set_normals(N_faces);

    viewer.launch();

    return 0;
}