#pragma once

#include <vector>
#include <Eigen/Eigen>

class ColorSetGenerator {
private:
    const int kMaxColorCount = 360;

public:
    explicit ColorSetGenerator(int colorCnt);
    const std::vector<Eigen::Vector3d>& GetColorSet() const;

private:
    Eigen::Vector3d HSL2RGB(float hue, float saturation, float lightness);
    float Hue2RGB(float p, float q, float t);

private:
    int m_colorCnt;
    std::vector<Eigen::Vector3d> m_colorSet;
};