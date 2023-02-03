#pragma once

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;

class IterativeCluster {
private:
    const int kInvalidClusterId = -1;

public:
    IterativeCluster(Mesh& mesh, OpenMesh::FProp<int>& clusterProp);
    void Run(int k, int maxIter = 100);
    Mesh::FaceHandle RegionGrow();
    float CalculateCost(const Mesh::FaceHandle& lhs, const Mesh::FaceHandle& rhs);
    void InitSeed();
    void ClearClusterProp();
    void UpdateClusterCenters();

private:

private:
    Mesh& m_mesh;
    float m_lambda;
    int m_clusterCount = 0;
    OpenMesh::FProp<int>& m_clusterProp;
    std::vector<Mesh::FaceHandle> m_seeds;
};