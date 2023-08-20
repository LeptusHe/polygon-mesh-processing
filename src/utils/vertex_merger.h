#pragma once
#include "utils/mesh_io.h"

class vertex_merger {
private:

public:
    CMesh Merge(CMesh& mesh);


private:
    void CollectMeshData(const CMesh& mesh);
    void MergeVertex();
    //Mesh RebuildMesh();

private:
    std::vector<CMesh::Point> m_vertices;
    std::vector<CMesh::Point> m_newVertices;
    std::vector<std::vector<std::size_t>> m_indices;
};


int RemoveDuplicationVertex(std::vector<CMesh::Point>& points, std::vector<std::vector<std::size_t>>& polygons);
int FixInvalidOrientation(std::vector<CMesh::Point>& points, std::vector<std::vector<std::size_t>>& polygons);
std::vector<CMesh::Point> SortPoints(const std::vector<CMesh::Point>& input);
void PrintSortedPoints(const std::vector<CMesh::Point>& points);
