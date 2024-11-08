#include "line.h"

namespace meshlib {

glm::vec3 Line::GetDir() const
{
    return p1 - p0;
}

} // namespace meshlib