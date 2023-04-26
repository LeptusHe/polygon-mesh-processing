#include "interfaces.h"
#include "common.h"
#include "iterative-cluster.h"
#include <cmath>

Mesh ConstructMesh(const float *vertices, int numVertices, const int *indices, int triangleNum)
{
    assert(vertices != nullptr && indices != nullptr);

    Mesh mesh;
    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();
    mesh.request_face_normals();

    for (int i = 0; i < numVertices; i++)
    {
        mesh.add_vertex(Mesh::Point(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]));
    }

    int numIndices = 3 * triangleNum;
    for (int i = 0; i < numIndices; i += 3)
    {
        mesh.add_face(mesh.vertex_handle(indices[i]), mesh.vertex_handle(indices[i + 1]), mesh.vertex_handle(indices[i + 2]));
    }

    mesh.update_face_normals();
    return mesh;
}

bool GetSegmentationResult(Mesh& mesh, OpenMesh::FProp<int>& clusterProp, int *clusterIds)
{
    if (clusterIds == nullptr)
        return false;

    for (auto faceHandle : mesh.faces())
    {
        clusterIds[faceHandle.idx()] = clusterProp[faceHandle];
    }
    return true;
}

bool RunSegmentation(const Options& options, Mesh& mesh, int *clusterIds)
{
    if (clusterIds == nullptr)
        return false;

    IterativeCluster::Options opt;
    //opt.maxChartArea = std::max(0.0f, options.maxChartArea);

    //OpenMesh::FProp<int> clusterProp(mesh, "cluster");

    //IterativeCluster cluster(mesh, clusterProp, options);

    //cluster.Init()
    //cluster.Run();

    //return GetSegmentationResult(mesh, clusterProp, clusterIds);
    return true;
}

bool Segmentation(Options options, const float *vertices, int numVertices, const int *indices, int triangleNum, const int *clusterIds)
{
    if (vertices == nullptr || indices == nullptr || clusterIds == nullptr)
        return false;

    auto mesh = ConstructMesh(vertices, numVertices, indices, triangleNum);
    //RunSegmentation(mesh, clusterIds);
    return true;
}