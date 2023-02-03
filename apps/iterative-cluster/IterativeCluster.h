#pragma once

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;

class IterativeCluster {
private:
    const int kInvalidClusterId = -1;

public:
    IterativeCluster(Mesh& mesh, OpenMesh::FProp<int>& clusterProp);
    void Run(int k, float lambda, int maxIter = 100);
    Mesh::FaceHandle RegionGrow();
    float CalculateCost(const Mesh::Normal& chartNormal, const Mesh::FaceHandle& oldFace, const Mesh::FaceHandle& newFace);
    void InitSeed();
    void ClearClusterProp();
    void UpdateClusterCenters();
    Mesh GetClusterMesh(int clusterId);
    bool IsConverged() const;

private:

private:
    Mesh& m_mesh;
    float m_lambda = 1;
    int m_clusterCount = 0;

    OpenMesh::FProp<int>& m_clusterProp;
    std::vector<Mesh::Normal> m_clusterNormals;

    std::vector<Mesh::FaceHandle> m_prevSeeds;
    std::vector<Mesh::FaceHandle> m_seeds;
};