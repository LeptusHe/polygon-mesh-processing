#include "vertex_merger.h"

namespace {

// https://github.com/g-truc/glm/blob/5c46b9c07008ae65cb81ab79cd677ecc1934b903/glm/gtx/hash.inl#L6
void hash_combine(size_t& seed, size_t hash)
{
    hash += 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hash;
}

constexpr float maxDigits = 100.0f;

class PointHash {
public:
    std::size_t operator()(const Mesh::Point& p) const
    {
        auto x = static_cast<int64_t >(p[0] * maxDigits);
        auto y = static_cast<int64_t>(p[1] * maxDigits);
        auto z = static_cast<int64_t>(p[2] * maxDigits);

        size_t seed = 0;
        std::hash<size_t> hasher;

        hash_combine(seed, hasher(x));
        hash_combine(seed, hasher(y));
        hash_combine(seed, hasher(z));

        return seed;
    }
};


class PointCompare {
public:
    bool operator()(const Mesh::Point& lhs, const Mesh::Point& rhs) const
    {
        auto lhs_x = static_cast<int64_t>(lhs[0] * maxDigits);
        auto lhs_y = static_cast<int64_t>(lhs[1] * maxDigits);
        auto lhs_z = static_cast<int64_t>(lhs[2] * maxDigits);

        auto rhs_x = static_cast<int64_t>(rhs[0] * maxDigits);
        auto rhs_y = static_cast<int64_t>(rhs[1] * maxDigits);
        auto rhs_z = static_cast<int64_t>(rhs[2] * maxDigits);

        return (lhs_x == rhs_x) && (lhs_y == rhs_y) && (lhs_z == rhs_z);
    }
};

}


Mesh vertex_merger::Merge(Mesh& mesh)
{
    CollectMeshData(mesh);
    MergeVertex();
    return RebuildMesh();
}

void vertex_merger::CollectMeshData(const Mesh& mesh)
{
    m_vertices.resize(mesh.n_vertices());
    for (auto vertex : mesh.vertices()) {
        auto idx = vertex.idx();
        auto p = mesh.point(vertex);
        m_vertices[idx] = p;
    }

    m_indices.resize(3 * mesh.n_faces());
    for (auto fh : mesh.faces()) {
        auto faceIndex = fh.idx();

        int vIdx = 0;
        for (auto vh : fh.vertices()) {
            m_indices[3 * faceIndex + vIdx] = vh.idx();
            vIdx += 1;
        }
    }
}

void vertex_merger::MergeVertex()
{
    m_newVertices.clear();
    std::unordered_map<Mesh::Point, int, PointHash, PointCompare> map;

    for (int i = 0; i < m_indices.size(); ++ i) {
        auto index = m_indices[i];
        auto vertex = m_vertices[index];

        if (map.find(vertex) == std::end(map)) {
            int newIndex = static_cast<int>(m_newVertices.size());
            m_newVertices.push_back(vertex);
            m_indices[i] = newIndex;

            map[vertex] = newIndex;
        } else {
            int newIndex = map[vertex];
            m_indices[i] = newIndex;
        }
    }
}

Mesh vertex_merger::RebuildMesh()
{
    Mesh mesh;
    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();
    mesh.request_face_normals();

    std::vector<Mesh::VertexHandle> vh_list(m_newVertices.size());
    for (int i = 0; i < m_newVertices.size(); i++) {
        auto vertex = m_newVertices[i];
        auto vh = mesh.add_vertex(vertex);
        vh_list[i] = vh;
    }

    for (int i = 0; i < m_indices.size(); i += 3) {
        mesh.add_face({
            vh_list[m_indices[i + 0]],
            vh_list[m_indices[i + 1]],
            vh_list[m_indices[i + 2]]
        });

        //mesh.add_face(mesh.vertex_handle(m_indices[i]),
        //              mesh.vertex_handle(m_indices[i + 1]),
        //              mesh.vertex_handle(m_indices[i + 2]));
    }
    mesh.update_face_normals();
    return mesh;
}