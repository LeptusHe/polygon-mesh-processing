#pragma once
#include "utils/mesh_io.h"

class vertex_merger {
private:

public:
    Mesh Merge(Mesh& mesh);

private:
    void CollectMeshData(const Mesh& mesh);
    void MergeVertex();
    Mesh RebuildMesh();
    void OutputSortedVertex(const std::vector<Mesh::Point>& points);

private:
    std::vector<Mesh::Point> m_vertices;
    std::vector<Mesh::Point> m_newVertices;
    std::vector<int> m_indices;
};
