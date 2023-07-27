#pragma once
#include "utils/mesh_io.h"

class vertex_merger {
public:
    void Merge(Mesh& mesh);
    void MergeVertex(Mesh::VertexHandle oldVertex, Mesh::VertexHandle newVertex)
};
