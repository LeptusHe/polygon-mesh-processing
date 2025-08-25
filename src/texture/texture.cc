#include "texture.h"
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include <spdlog/spdlog.h>

namespace meshlib {

Texture::~Texture()
{
    stbi_image_free(data_);
}

bool Texture::Load(const std::string& file_path)
{
    file_path_ = file_path;

    data_ = stbi_load(file_path_.c_str(), &width_, &height_, &n_channel_, 0);
    if (!data_) {
        spdlog::error("failed to load image, path=[{}]", file_path);
        return false;
    }

    spdlog::info("succeed to load image, path=[{}]", file_path);
    return true;
}

Eigen::Vector3f Texture::Sample(const Mesh::TexCoord2D& uv) const
{
    float s = std::fmod(uv[0], 1.0f) * static_cast<float>(width_);
    while (s < 0) {
        s += static_cast<float>(width_);
    }

    float t = std::fmod(uv[1], 1.0f) * static_cast<float>(height_);
    while (t < 0) {
        t += static_cast<float>(height_);
    }

    const auto i = std::clamp(static_cast<int>(std::floor(s)), 0, width_ - 1);
    const auto j = std::clamp(static_cast<int>(std::floor(t)), 0, height_ - 1);
    const auto ii = (i + 1) % width_;
    const auto jj = (j + 1) % height_;

    const auto u = s - static_cast<float>(i);
    const auto v = t - static_cast<float>(j);

    return (1 - u) * (1 - v) * GetPixel(i, j) +
        (1 - u) * v * GetPixel(i, jj) +
        u * (1 - v) * GetPixel(ii, j) +
        u * v * GetPixel(ii, jj);
}

Eigen::Vector3f Texture::GetPixel(int w_index, int h_index) const
{
    const auto bytes_per_pixel = n_channel_;

    const auto index = (h_index * width_ + w_index) * bytes_per_pixel;
    const uint8_t x = n_channel_ >= 1 ? data_[index + 0] : 0;
    const uint8_t y = n_channel_ >= 2 ? data_[index + 1] : 0;
    const uint8_t z = n_channel_ >= 3 ? data_[index + 2] : 0;

    return {
        static_cast<float>(x) / 255.0f,
        static_cast<float>(y) / 255.0f,
        static_cast<float>(z) / 255.0f
    };
}

} // namespace meshlib