#include "uv-clipper.h"

namespace meshlib {

int calculate_offset_to_range_0_and_1(float val)
{

}


void UVClipper::process(Mesh& mesh)
{
    for (const auto face : mesh.faces()) {
        std::vector<Mesh::VertexHandle> vh_list;
        for (const auto vh : mesh.fv_range(face)) {
            vh_list.push_back(vh);
        }
    }
}

void UVClipper::process_triangle(std::vector<Mesh::VertexHandle>& triangle)
{
    const auto uv_offset = calculate_uv_offset(triangle);
    for (const auto vh : triangle) {
        auto p = mesh.point(vh);
        p[0] += uv_offset.x;
        p[1] += uv_offset.y;

        mesh.set_point(vh, p);
    }
}

glm::ivec2 UVClipper::calculate_uv_offset(const std::vector<Mesh::VertexHandle>& triangle)
{
    Mesh::VertexHandle vh_y_min = triangle[0];
    for (const auto vh : triangle) {
        auto p = mesh.point(vh);
        auto p_y_min = mesh.point(vh_y_min);

        if (p[2] < p_y_min[2]) {
            vh_y_min = vh;
        }
    }

    auto p = mesh.point(vh_y_min);

    int x_offset = -std::floor(p[0]);
    int y_offset = -std::floor(p[1]);

    return {x_offset, y_offset};
}

std::vector<Mesh::VertexHandle> UVClipper::clip_polygon_by_uv_bounds(std::vector<Mesh::VertexHandle>& polygon, const Line& line)
{
    std::vector<Mesh::VertexHandle> result;

    const auto vertex_cnt = polygon.size();
    for (int cur_index = 0; cur_index < vertex_cnt; ++ i) {
        const auto prev_index = (cur_index + vertex_cnt - 1) % vertex_cnt;

        const auto prev_vh = polygon[prev_index];
        const auto cur_vh = polygon[cur_index];

        const auto prev_p = mesh.point(prev_vh);
        const auto cur_p = mesh.point(cur_vh);

        const auto prev_p_inside = is_inside(prev_p, line);
        const auto cur_p_inside = is_inside(cur_p, line);

        if (cur_p_inside) {
            if (!prev_p_inside) {
                float t = 0.0f;


            } else {
                result.push_back(cur_vh);
            }
        } else {

        }

    }
}

bool UVClipper::is_inside(const Mesh::Point& v, const Line& line)
{
    const auto p = convert_to(v);
    const auto p0p = p - line.p0;
    const auto p0p1 = line.GetDir();

    // Note: 如果三角形的某个顶点在直线上，会被判定为在直线的外侧，从而删除多个相同的相交点
    return p0p.x * p0p1.z - p0p.z * p0p1.x >= 0;
}

Mesh::Point UVClipper::convert_to_point(const glm::vec3& p)
{
    return Mesh::Point(p.x, p.y, p.z);
}

glm::vec3 UVClipper::convert_to(const Mesh::Point& p)
{
    return glm::vec3(p[0], p[1], p[2]);
}

} // namespace meshlib