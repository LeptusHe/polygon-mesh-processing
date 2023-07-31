#include "vertex_merger.h"

namespace {

// https://github.com/g-truc/glm/blob/5c46b9c07008ae65cb81ab79cd677ecc1934b903/glm/gtx/hash.inl#L6
void hash_combine(size_t& seed, size_t hash) const
{
    hash += 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hash;
}

}


void vertex_merger::Merge(Mesh& mesh)
{
    OpenMesh::VPropHandleT<bool> processed;
    mesh.add_property(processed, "processed");

    for (const auto& vh1 : mesh.vertices()) {
        if (!mesh.property(processed, vh1)) {
            for (const auto& vh2 : mesh.vertices()) {
                if (vh1 != vh2 && !mesh.property(processed, vh2)) {
                    Mesh::Point p1 = mesh.point(vh1);
                    Mesh::Point p2 = mesh.point(vh2);

                    double distance = (p1 - p2).norm();
                    if (distance <= distance_threshold) {
                        // 合并位置距离接近的顶点
                        mesh.property(processed, vh2) = true;
                        mesh.collapse_vertex(vh2, vh1);
                    }
                }
            }
        }
    }

    // 删除已处理的顶点属性
    mesh.remove_property(processed);

    // 执行垃圾回收
    mesh.garbage_collection();
}

void vertex_merger::CollectMeshData(const Mesh& mesh)
{
    vertices.resize(mesh.n_vertices());
    for (auto vertex : mesh.vertices()) {
        auto idx = vertex.idx();
        auto p = mesh.point(vertex);
        vertices[idx] = p;
    }

    indices.resize(3 * mesh.n_faces());
    for (auto fh : mesh.faces()) {
        auto faceIndex = fh.idx();

        int vIdx = 0;
        for (auto vh : fh.vertices()) {
            indices[3 * faceIndex + vIdx] = vh.idx();
            vIdx += 1;
        }
    }
}


std::size_t vertex_merger::hash(const Mesh::Point& p) const
{
    auto x = static_cast<size_t>(p[0] * maxDigits);
    auto y = static_cast<size_t>(p[1] * maxDigits);
    auto z = static_cast<size_t>(p[2] * maxDigits);

    size_t seed = 0;
    std::hash<size_t> hasher;

    hash_combine(seed, hasher(x));
    hash_combine(seed, hasher(y));
    hash_combine(seed, hasher(z));

    return seed;
}

bool vertex_merger::equal(const Mesh::Point& lhs, const Mesh::Point& rhs) const
{
    auto lhs_x = static_cast<size_t>(lhs[0] * maxDigits);
    auto lhs_y = static_cast<size_t>(lhs[1] * maxDigits);
    auto lhs_z = static_cast<size_t>(lhs[2] * maxDigits);

    auto rhs_x = static_cast<size_t>(rhs[0] * maxDigits);
    auto rhs_y = static_cast<size_t>(rhs[1] * maxDigits);
    auto rhs_z = static_cast<size_t>(rhs[2] * maxDigits);

    return (lhs_x == rhs_x) && (lhs_y == rhs_y) && (lhs_z == rhs_z);
}