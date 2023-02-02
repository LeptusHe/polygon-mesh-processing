#pragma once

#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;

class KMeans {
public:
    KMeans(Mesh& mesh, OpenMesh::VProp<int>& clusterProp)
        : m_mesh(mesh), m_clusterProp(clusterProp) {}
    ~KMeans() = default;;
    void Run(int k, int maxIter = 100);

private:
    int GetClusterId(const Mesh::Point& p) const;
    void UpdateClusterCenters();
    bool IsConverged() const;

private:
    Mesh& m_mesh;
    OpenMesh::VProp<int>& m_clusterProp;

    int m_clusterCount = 0;
    std::vector<Mesh::VertexHandle> m_seeds;
    std::vector<Mesh::Point> m_prevClusterCenters;
    std::vector<Mesh::Point> m_clusterCenters;
};