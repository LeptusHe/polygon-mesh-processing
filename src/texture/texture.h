#pragma once

#include <string>
#include <Eigen/Eigen>

#include "utils/mesh_io.h"

namespace meshlib {

class Texture {
public:
    ~Texture();

    bool Load(const std::string& file_path);
    Eigen::Vector3f Sample(const Mesh::TexCoord2D& uv) const;

private:
    [[nodiscard]] Eigen::Vector3f GetPixel(int width, int height) const;

private:
    std::string file_path_;

    uint8_t *data_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    int n_channel_ = 0;
};

} // namespace meshlib
