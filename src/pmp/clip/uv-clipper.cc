#include <iostream>
#include "uv-clipper.h"
#include "geometry/intersection.h"

namespace meshlib {

namespace {

template <typename T>
T Lerp(const T& a, const T& b, float t)
{
    t = glm::clamp(t, 0.0f, 1.0f);
    return a + t * (b - a);
}

}


void UVClipper::process(Mesh& mesh)
{
    init_mesh(mesh);

    for (const auto face : mesh.faces()) {
        std::vector<Mesh::VertexHandle> vh_list;
        for (const auto vh : mesh.fv_range(face)) {
            vh_list.push_back(vh);
        }

        process_triangle(vh_list);
    }
}

void UVClipper::init_mesh(Mesh& mesh)
{
    m_mesh = mesh;

    if (mesh.has_vertex_texcoords2D()) {
        m_clipped_mesh.request_vertex_texcoords2D();
    }
}

void UVClipper::process_triangle(const std::vector<Mesh::VertexHandle>& triangle)
{
    const auto uv_bounds = calculate_uv_bounds(triangle);
    const glm::ivec2 uv_min = uv_bounds.min();
    const glm::ivec2 uv_max = uv_bounds.max();

    for (int y = uv_min.y; y < uv_max.y; ++ y) {
        for (int x = uv_min.x; x < uv_max.x; ++ x) {
            glm::ivec2 sub_bounds_min{x, y};
            glm::ivec2 sub_bounds_max{x + 1, y + 1};

            const auto sub_bounds = Bounds2D::FromMinAndMax(sub_bounds_min, sub_bounds_max);
            const auto polygon = clip_polygon_by_uv_bounds(triangle, sub_bounds);
            triangulate_polygon(polygon);
        }
    }
}

std::vector<Mesh::VertexHandle> UVClipper::clip_polygon_by_uv_bounds(const std::vector<Mesh::VertexHandle>& polygon,
                                                                     const Bounds2D& bounds)
{
    auto polygon_vh_list = polygon;
    const auto lines = generate_bound_lines(bounds);
    for (const auto& line : lines) {
        polygon_vh_list = get_clipped_polygon_by_line(polygon_vh_list, line);
    }
    return polygon_vh_list;
}


std::vector<Mesh::VertexHandle> UVClipper::get_clipped_polygon_by_line(const std::vector<Mesh::VertexHandle>& polygon,
                                                                       const Line2D& line)
{
    std::vector<Mesh::VertexHandle> result;

    const auto vertex_cnt = polygon.size();
    for (int cur_index = 0; cur_index < vertex_cnt; ++ cur_index) {
        const auto prev_index = (cur_index + vertex_cnt - 1) % vertex_cnt;

        const auto prev_vh = polygon[prev_index];
        const auto cur_vh = polygon[cur_index];

        const auto prev_p = get_uv(prev_vh);
        const auto cur_p = get_uv(cur_vh);

        const auto prev_p_inside = is_inside(prev_p, line);
        const auto cur_p_inside = is_inside(cur_p, line);

        if (cur_p_inside) {
            if (!prev_p_inside) {
                float t = calculate_intersection_point(prev_p, cur_p, line);
                const auto intersection_vh = add_intersection_point(t, prev_vh, cur_vh);

                result.push_back(intersection_vh);
                result.push_back(cur_vh);
            } else {
                result.push_back(cur_vh);
            }
        } else {
            if (prev_p_inside) {
                float t = calculate_intersection_point(prev_p, cur_p, line);
                const auto intersection_vh = add_intersection_point(t, prev_vh, cur_vh);
                result.push_back(intersection_vh);
            }
        }
    }
    return result;
}

Bounds<glm::vec2> UVClipper::calculate_uv_bounds(const std::vector<Mesh::VertexHandle>& triangle)
{
    glm::vec2 min_uv{std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    glm::vec2 max_uv{std::numeric_limits<float>::min(), std::numeric_limits<float>::min()};

    for (const auto vh : triangle) {
        const auto uv = get_uv(vh);

        min_uv = glm::min(uv, min_uv);
        max_uv = glm::max(uv, max_uv);
    }

    min_uv = glm::floor(min_uv);
    max_uv = glm::ceil(max_uv);

    return Bounds<glm::vec2>::FromMinAndMax(min_uv, max_uv);
}

std::vector<Line2D> UVClipper::generate_bound_lines(const Bounds2D& bounds)
{
    const auto min_v = bounds.min();
    const auto max_v = bounds.max();

    const auto p0 = glm::vec2(min_v.x, min_v.y);
    const auto p1 = glm::vec2(min_v.x, max_v.y);
    const auto p2 = glm::vec2(max_v.x, max_v.y);
    const auto p3 = glm::vec2(max_v.x, min_v.y);

    const auto line0 = Line2D(p0, p1);
    const auto line1 = Line2D(p1, p2);
    const auto line2 = Line2D(p2, p3);
    const auto line3 = Line2D(p3, p0);

    const auto lines = std::vector<Line2D> {
        line0,
        line1,
        line2,
        line3
    };
    return lines;
}

float UVClipper::calculate_intersection_point(const glm::vec2& p0, const glm::vec2& p1, const Line2D& line)
{
    LineSegment2D line_seg{p0, p1};
    const auto t = calc_2d_line_and_segment_intersection(line, line_seg);
    assert(t >= 0 && t <= 1);
    return t;
}

bool UVClipper::is_inside(const glm::vec2& p, const Line2D& line)
{
    const auto p0p = p - line.p0;
    const auto p0p1 = line.GetDir();

    // Note: 如果三角形的某个顶点在直线上，会被判定为在直线的外侧，从而删除多个相同的相交点
    return p0p.x * p0p1.y - p0p.y * p0p1.x >= 0;
}

glm::vec2 UVClipper::get_uv(const Mesh::VertexHandle& vh)
{
    const auto uv = m_mesh.texcoord2D(vh);
    return {uv[0], uv[1]};
}

Mesh::VertexHandle UVClipper::add_intersection_point(float t, const Mesh::VertexHandle& v0, const Mesh::VertexHandle& v1)
{
    const auto p0 = m_mesh.point(v0);
    const auto p1 = m_mesh.point(v1);

    const auto p = Lerp(p0, p1, t);
    const auto vh = m_mesh.add_vertex(p);

    const auto uv = Lerp(m_mesh.texcoord2D(v0), m_mesh.texcoord2D(v1), t);
    m_mesh.set_texcoord2D(vh, uv);

    // TODO: normal, tangent
    return vh;
}

void UVClipper::triangulate_polygon(const std::vector<Mesh::VertexHandle>& polygon)
{
    if (polygon.size() == 0)
        return;

    if (polygon.size() < 3) {
        std::cerr << "invalid vertex count of polygon is " << polygon.size() << std::endl;
        return;
    }

    std::vector<Mesh::VertexHandle> new_polygon;
    for (const auto vh: polygon) {
        const auto new_vh = copy_vertex_to_clipped_mesh(vh);
        new_polygon.push_back(new_vh);
    }

    const auto v0 = new_polygon[0];
    for (int i = 2; i < new_polygon.size(); ++ i) {
        const auto v1 = new_polygon[i - 1];
        const auto v2 = new_polygon[i - 0];

        std::vector<Mesh::VertexHandle> face {
            v0,
            v1,
            v2
        };
        m_clipped_mesh.add_face(face);
    }
}

Mesh::VertexHandle UVClipper::copy_vertex_to_clipped_mesh(const Mesh::VertexHandle& vh)
{
    auto p = m_mesh.point(vh);
    auto v0 = m_clipped_mesh.add_vertex(p);

    auto uv = m_mesh.texcoord2D(vh);
    m_clipped_mesh.set_texcoord2D(v0, uv);
    return v0;
}

const Mesh& UVClipper::get_clipped_mesh() const
{
    return m_clipped_mesh;
}

} // namespace meshlib