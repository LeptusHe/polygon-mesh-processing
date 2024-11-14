#pragma once

#include "utils/mesh_io.h"
#include <glm/glm.hpp>

namespace meshlib {

template <typename T>
class Line {
public:
    Line(const T& p0, const T& p1) : p0(p0), p1(p1) {}

    [[nodiscard]] T GetDir() const
    {
        return p1 - p0;
    }

public:
    T p0;
    T p1;
};


template <typename T>
class LineSegment {
public:
    LineSegment(const T& start, const T& end): start(start), end(end) {}

public:
    T start;
    T end;
};


using Line2D = Line<glm::vec2>;
using LineSegment2D = LineSegment<glm::vec2>;

} // namespace meshlib
