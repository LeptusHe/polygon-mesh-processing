#include "sampler.h"
#include <random>

namespace meshlib {

class UniformSampler : Sampler {
public:
    UniformSampler()
    {
        std::random_device rd;

        gen_ = std::mt19937(rd());
        dist_ = std::uniform_real_distribution<float>(0.0f, 1.0f);
    }

    float Get1D() override
    {
        return dist_(gen_);
    }

    Eigen::Vector2f Get2D() override
    {
        const auto x = dist_(gen_);
        const auto y = dist_(gen_);
        return {x, y};
    }

private:
    std::mt19937 gen_;
    std::uniform_real_distribution<float> dist_;
};


} // namespace meshlib