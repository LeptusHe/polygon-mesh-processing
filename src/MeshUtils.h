#pragma once
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <igl/opengl/glfw/Viewer.h>

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;

namespace meshlib {

class MeshUtils {
public:
    static void ConvertMeshToViewer(const Mesh& mesh, igl::opengl::glfw::Viewer& viewer);
};

} // namespace meshlib
