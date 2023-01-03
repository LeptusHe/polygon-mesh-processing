#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>

int main(int argc, char *argv[])
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    auto path = argc > 1 ? argv[1] : "data/bunny.off";
    if (!igl::read_triangle_mesh(path, V, F)) {

    }



    return 0;
}