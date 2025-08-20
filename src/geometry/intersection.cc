#include "intersection.h"

namespace meshlib {

// calculate intersect point of line and segment in y=0 plane
// ref: https://gamedev.stackexchange.com/questions/44720/line-intersection-from-parametric-equation
float calc_2d_line_and_segment_intersection(const Line2D& line, const LineSegment2D& segment)
{
    const auto a = segment.start;
    const auto b = segment.end - segment.start;

    const auto c = line.p0;
    const auto d = line.p1 - line.p0;

    const float t0 = d.x * (c.y - a.y);
    const float t1 = d.y * (a.x - c.x);
    const float t2 = b.y * d.x - b.x * d.y;

    const float t = (t0 + t1) / t2;
    return t;
}

} // namespace meshlib