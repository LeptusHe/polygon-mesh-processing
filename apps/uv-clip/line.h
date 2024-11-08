#pragma once

#include "utils/mesh_io.h"
#include <glm/glm.hpp>

namespace meshlib {

class Line {
public:
    glm::vec3 GetDir() const;

public:
    glm::vec3 p0;
    glm::vec3 p1;
};

} // namespace meshlib
