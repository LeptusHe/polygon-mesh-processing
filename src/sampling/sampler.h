#pragma once

#include <Eigen/Eigen>

namespace meshlib {

class Sampler {
public:
    virtual ~Sampler() = default;

    virtual float Get1D() = 0;
    virtual Eigen::Vector2f Get2D() = 0;
};

} // namespace meshlib