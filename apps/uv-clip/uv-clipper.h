#pragma once

#include "utils/mesh_io.h"
#include <glm/glm.hpp>
#include "geometry/line.h"
#include "geometry/bounds.h"

namespace meshlib {

class UVClipper {
public:
    void process(Mesh& mesh);
    void process_triangle(const std::vector<Mesh::VertexHandle>& triangle);

private:
    std::vector<Mesh::VertexHandle> clip_polygon_by_uv_bounds(const std::vector<Mesh::VertexHandle>& polygon,
                                                              const Bounds2D& bounds);

    std::vector<Mesh::VertexHandle> get_clipped_polygon_by_line(const std::vector<Mesh::VertexHandle>& polygon,
                                                                const Line2D& line);

    bool is_inside(const glm::vec2& uv, const Line2D& line);
    float calculate_intersection_point(const glm::vec2& p0, const glm::vec2& p1, const Line2D& line);
    Bounds<glm::vec2> calculate_uv_bounds(const std::vector<Mesh::VertexHandle>& triangle);
    std::vector<Line2D> generate_bound_lines(const Bounds2D& bounds);

private:
    Mesh::VertexHandle add_intersection_point(float t, const Mesh::VertexHandle& v0, const Mesh::VertexHandle& v1);
    glm::vec2 get_uv(const Mesh::VertexHandle& vh);

private:
    Mesh m_mesh;
};

} // namespace meshlib