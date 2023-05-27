#include "interfaces.h"
#include "common.h"
#include "iterative-cluster.h"
#include <iostream>

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

IterativeCluster::Options InitOptions(const Options& options)
{
    IterativeCluster::Options opt;
    opt.minClusterCnt = std::max(1, options.minClusterCount);
    opt.maxChartArea = std::max(0.0f, options.maxChartArea);
    opt.normalWeight = std::max(0.0f, options.normalWeight);
    opt.maxIterationNum = std::max(1, options.maxIterationNum);
    return opt;
}

bool RunSegmentation(const Options& options, Mesh& mesh, int *clusterIds, int& clusterCnt)
{
    if (clusterIds == nullptr)
        return false;

    auto opt = InitOptions(options);
    OpenMesh::FProp<int> clusterProp(mesh, "cluster");

    try {
        IterativeCluster cluster(mesh, clusterProp);
        cluster.Run(opt);

        clusterCnt = static_cast<int>(cluster.GetClusterCount());
    } catch (std::exception& e) {
        std::cerr << "failed to run segmentation: " << e.what() << std::endl;
        clusterCnt = -1;
        return false;
    }

    return GetSegmentationResult(mesh, clusterProp, clusterIds);
}

int MeshSegmentation(Options options, const float vertices[], int numVertices, const int indices[], int triangleNum, int clusterIds[])
{
    if (vertices == nullptr || indices == nullptr || clusterIds == nullptr)
        return false;

    auto mesh = ConstructMesh(vertices, numVertices, indices, triangleNum);


    int clusterCnt = -1;
    auto success = RunSegmentation(options, mesh, clusterIds, clusterCnt);
    if (success) {
        return clusterCnt;
    } else {
        return -1;
    }
}