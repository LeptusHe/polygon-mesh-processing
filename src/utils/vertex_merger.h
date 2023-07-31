#pragma once
#include "utils/mesh_io.h"

class vertex_merger {
private:
    static constexpr float maxDigits = 1000000.0f;

public:
    void Merge(Mesh& mesh);
    void MergeVertex(Mesh::VertexHandle oldVertex, Mesh::VertexHandle newVertex);

private:
    void CollectMeshData(const Mesh& mesh);
    [[nodiscard]] std::size_t hash(const Mesh::Point& p) const;
    [[nodiscard]] bool equal(const Mesh::Point& lhs, const Mesh::Point& rhs) const;

private:
    std::vector<Mesh::Point> vertices;
    std::vector<int> indices;
};
