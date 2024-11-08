#pragma once

#include "utils/mesh_io.h"
#include <glm/glm.hpp>
#include "line.h"

namespace meshlib {

class UVClipper {
public:
    void process(Mesh& mesh);
    void process_triangle(Mesh& mesh, std::vector<Mesh::VertexHandle>& triangle);


private:
    glm::ivec2 calculate_uv_offset(const std::vector<Mesh::VertexHandle>& triangle);
    std::vector<Mesh::VertexHandle> clip_polygon_by_uv_bounds(std::vector<Mesh::VertexHandle>& polygon);
    std::vector<Mesh::VertexHandle> get_clipped_polygon_by_line(std::vector<Mesh::VertexHandle>& polygon);
    bool is_inside(const Mesh::Point& p, const Line& line);
    float calculate_intersection_point(const Mesh::Point& p0, const Mesh::Point& p1);

private:
    Mesh::Point convert_to_point(const glm::vec3& p);
    glm::vec3 convert_to(const Mesh::Point& p);

private:
    Mesh mesh;
};

} // namespace meshlib