#pragma once

#include <glm/glm.hpp>

class Bounds {
public:
    glm::vec2 min = glm::vec2();
    glm::vec2 max = glm::vec2();

    Bounds() = delete;

    explicit Bounds(const glm::vec2& center)
    {
        min = center;
        max = center;
    }

    void Encapsulate(const glm::vec2& p)
    {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);

        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
    }

    [[nodiscard]] glm::vec2 size() const
    {
        glm::vec2 size;
        size.x = std::abs(max.x - min.x);
        size.y = std::abs(max.y - min.y);
        return size;
    }

    [[nodiscard]] float area() const
    {
        const auto uv_size = size();
        return uv_size.x * uv_size.y;
    }

    [[nodiscard]] bool Inside(const Bounds& other) const
    {
        if (min.x < other.min.x)
            return false;

        if (min.y < other.min.y)
            return false;

        if (max.x > other.max.x)
            return false;

        if (max.y > other.max.y)
            return false;

        return true;
    }
};