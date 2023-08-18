#include "vertex_merger.h"
#include <spdlog/spdlog.h>

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
    std::size_t operator()(const CMesh::Point& p) const
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
    bool operator()(const CMesh::Point& lhs, const CMesh::Point& rhs) const
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


CMesh vertex_merger::Merge(CMesh& mesh)
{
    CollectMeshData(mesh);
    MergeVertex();
    //return RebuildMesh();
    return mesh;
}

void vertex_merger::CollectMeshData(const CMesh& mesh)
{
    m_vertices.resize(mesh.num_vertices());
    for (auto vertex : mesh.vertices()) {
        auto idx = vertex.idx();
        auto p = mesh.point(vertex);
        m_vertices[idx] = p;
    }

    m_indices.resize(mesh.num_faces());
    for (auto fh : mesh.faces()) {
        auto faceIndex = fh.idx();

        int vIdx = 0;
        std::vector<std::size_t> indices(3);
        for (const auto vh : mesh.vertices_around_face(mesh.halfedge(fh))) {
            indices.push_back(vh.idx());

            //m_indices[3 * faceIndex + vIdx] = vh.idx();
            //vIdx += 1;
        }
        m_indices[faceIndex] = indices;

        /*
        for (auto vh : fh.vertices()) {
            m_indices[3 * faceIndex + vIdx] = vh.idx();
            vIdx += 1;
        }
         */
    }
}

void vertex_merger::MergeVertex()
{
    m_newVertices.clear();
    std::unordered_map<CMesh::Point, int, PointHash, PointCompare> map;

    for (int i = 0; i < m_indices.size(); ++ i) {
        auto& polygon = m_indices[i];

        for (int k = 0; k < polygon.size(); ++ k) {
            auto index = polygon[k];

            auto point = m_vertices[index];

            if (map.find(point) == std::end(map)) {
                int newIndex = static_cast<int>(m_newVertices.size());

                m_newVertices.push_back(point);
                polygon[i] = newIndex;

                map[point] = newIndex;
            } else {
                int newIndex = map[point];
                polygon[i] = newIndex;
            }
        }

        //auto index = m_indices[i];
        /*
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
         */
    }
}

/*
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
 */

float Clamp(float a)
{
    constexpr float digit = 100.0f;

    float x = std::floor(a * digit);
    return x / digit;
}


int RemoveDuplicationVertex(std::vector<CMesh::Point>& points, std::vector<std::vector<std::size_t>>& polygons)
{
    std::vector<CMesh::Point> m_newVertices;
    std::unordered_map<CMesh::Point, int, PointHash, PointCompare> map;

    for (int i = 0; i < polygons.size(); ++ i) {
        auto& polygon = polygons[i];

        for (int k = 0; k < polygon.size(); ++ k) {
            auto index = polygon[k];

            auto point = points[index];

            if (map.find(point) == std::end(map)) {
                int newIndex = static_cast<int>(m_newVertices.size());

                m_newVertices.push_back(point);
                polygon[k] = newIndex;

                map[point] = newIndex;
            } else {
                int newIndex = map[point];
                polygon[k] = newIndex;
            }
        }
    }

    for (auto& p : m_newVertices) {
        p = {Clamp(p[0]), Clamp(p[1]), Clamp(p[2])};
    }

    int remove_cnt = points.size() - m_newVertices.size();
    points = m_newVertices;
    return remove_cnt;
}

std::vector<CMesh::Point> SortPoints(const std::vector<CMesh::Point>& input)
{
    auto points = input;
    std::sort(std::begin(points), std::end(points), [](const auto& lhs, const auto& rhs){
        return std::tie(lhs[0], lhs[2], lhs[1]) < std::tie(rhs[0], rhs[2], rhs[1]);
    });
    return points;
}

void PrintSortedPoints(const std::vector<CMesh::Point>& points)
{
    for (const auto p : points) {
        spdlog::info("{}, {}, {}", p[0], p[1], p[2]);
    }
}
