#pragma once

#include "utils/mesh_io.h"
#include <glm/glm.hpp>

namespace meshlib {

class Line {
public:
    Line(const glm::vec3& p0, const glm::vec3& p1) : p0(p0), p1(p1) {}
    [[nodiscard]] glm::vec3 GetDir() const;

public:
    glm::vec3 p0;
    glm::vec3 p1;
};


class LineSegment {
public:
    LineSegment(const glm::vec3& start, const glm::vec3& end): start(start), end(end) {}

public:
    glm::vec3 start;
    glm::vec3 end;
};

} // namespace meshlib
