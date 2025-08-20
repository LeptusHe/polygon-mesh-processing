#include <imgui.h>
#include "iterative-cluster.h"
#include "interactive-gui.h"

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;

int main(int argc, char *argv[])
{
    return interactive(argc, argv);
}