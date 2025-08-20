#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/component_wise.hpp>

namespace meshlib {

template <typename T>
class Bounds {
public:
    T center;
    T extents;

    Bounds() : center(0.0f), extents(0.0f) {}

    Bounds(const T& center, const T& size)
        : center(center), extents(0.5f * size) {}

    [[nodiscard]] T min() const
    {
        return center - extents;
    }

    [[nodiscard]] T max() const
    {
        return center + extents;
    }

    [[nodiscard]] T size() const
    {
        return extents * 2.0f;
    }

    void encapsulate(const T& point)
    {
        T min_point = glm::min(min(), point);
        T max_point = glm::max(max(), point);
        center = 0.5f * (min_point + max_point);
        extents = 0.5f * (max_point - min_point);
    }

    void encapsulate(const Bounds<T>& other)
    {
        encapsulate(other.min());
        encapsulate(other.max());
    }

    bool contains(const T& point) const
    {
        return glm::all(glm::lessThanEqual(glm::abs(point - center), extents));
    }

    bool intersects(const Bounds<T>& other) const
    {
        return glm::all(glm::lessThanEqual(glm::abs(other.center - center), extents + other.extents));
    }

public:
    static Bounds<T> FromMinAndMax(const T& min, const T& max)
    {
        const T center = static_cast<T>(0.5f * (min + max));
        const T size = max - min;

        return Bounds(center, size);
    }
};


using Bounds2D = Bounds<glm::vec2>;
using Bounds2I = Bounds<glm::ivec2>;

} // namespace meshlib
