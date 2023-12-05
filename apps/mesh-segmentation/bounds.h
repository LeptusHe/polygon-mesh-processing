#pragma once

#include <Eigen/Eigen>

class Bounds {
public:
    Eigen::Vector2f min = Eigen::Vector2f::Zero();
    Eigen::Vector2f max = Eigen::Vector2f::Zero();

    Bounds() = delete;

    explicit Bounds(const Eigen::Vector2f& center)
    {
        min = center;
        max = center;
    }

    void Encapsulate(const Eigen::Vector2f& p)
    {
        min.x() = std::min(min.x(), p.x());
        min.y() = std::min(min.y(), p.y());

        max.x() = std::max(max.x(), p.x());
        max.y() = std::max(max.y(), p.y());
    }

    [[nodiscard]] Eigen::Vector2f size() const
    {
        Eigen::Vector2f size;
        size.x() = std::abs(max.x() - min.x());
        size.y() = std::abs(max.y() - min.y());
        return size;
    }

    [[nodiscard]] float area() const
    {
        const auto uv_size = size();
        return uv_size.x() * uv_size.y();
    }

    [[nodiscard]] bool Inside(Bounds other) const
    {
        if (min.x() < other.min.x())
            return false;

        if (min.y() < other.min.y())
            return false;

        if (max.x() > other.max.x())
            return false;

        if (max.y() > other.max.y())
            return false;

        return true;
    }
};