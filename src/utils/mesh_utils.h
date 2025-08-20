#pragma once
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <igl/opengl/glfw/Viewer.h>

#include <CGAL/Cartesian.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;

namespace pmp = CGAL::Polygon_mesh_processing;

//using Kernal = CGAL::Exact_predicates_inexact_constructions_kernel;
using Kernel = CGAL::Simple_cartesian<float>;
using CMesh = CGAL::Surface_mesh<Kernel::Point_3>;

namespace meshlib {

class MeshUtils {
public:
    static void ConvertMeshToViewer(const Mesh& mesh, igl::opengl::glfw::Viewer& viewer);
    static void ConvertMeshToViewer(const CMesh& mesh, igl::opengl::glfw::Viewer& viewer);
    static bool WriteMesh(const Mesh& mesh, const std::string& path);
    static std::vector<CMesh::Point> GetSortedPoints(const CMesh& cmesh);
    static std::vector<CMesh::Point> GetSortedPoints(const std::vector<CMesh::Point>& points);
};

void CollectMeshData(const Mesh& mesh, std::vector<float>& vertices, std::vector<int>& indices);
CMesh ConstructMesh(const float *vertices, const float* normals, int numVertices, const int *indices, int triangleNum);
int CheckInvalidEdges(CMesh& clean_mesh);
Mesh ConvertToMesh(const CMesh& cmesh);
CMesh ConvertOpenMeshToSurfaceMesh(const Mesh& mesh);

} // namespace meshlib
