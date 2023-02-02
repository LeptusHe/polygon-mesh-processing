#include "ColorSetGenerator.h"
#include <random>

ColorSetGenerator::ColorSetGenerator(int colorCnt)
    : m_colorCnt(colorCnt)
{
    m_colorSet.reserve(colorCnt);
    m_colorCnt = std::min(colorCnt, kMaxColorCount);

    std::random_device rd;
    std::uniform_real_distribution<float> dist(0, 1);

    for (int i = 0; i < 360; i += 360 / m_colorCnt) {
        auto hue = (float)i;
        float saturation = 240.0f + (float)dist(rd)  * 10.0f;
        float lightness = 160.0f + (float)dist(rd) * 30.0f;
        auto color = HSL2RGB(hue, saturation, lightness);

        float val = 1.0;
        color[0] = std::pow(color[0], val);
        color[1] = std::pow(color[1], val);
        color[2] = std::pow(color[2], val);

        m_colorSet.push_back(color);
    }
}

const std::vector<Eigen::Vector3d>& ColorSetGenerator::GetColorSet() const
{
    return m_colorSet;
}

Eigen::Vector3d ColorSetGenerator::HSL2RGB(float hue, float saturation, float lightness)
{
    hue /= 256.0f;
    saturation /= 256.0f;
    lightness /= 256.0f;
    float b;
    float r;
    float g;

    if (saturation == 0.0f) {
        g = (r = (b = lightness));
    } else {
        float q = ((double)lightness < 0.5) ? (lightness * (1.0f + saturation)) : (lightness + saturation - lightness * saturation);
        float p = 2.0f * lightness - q;
        r = Hue2RGB(p, q, hue + 0.33333334f);
        g = Hue2RGB(p, q, hue);
        b = Hue2RGB(p, q, hue - 0.33333334f);
    }
    return {r, g, b};
}

float ColorSetGenerator::Hue2RGB(float p, float q, float t)
{
    if (t < 0.0f) {
        t += 1.0f;
    }

    if (t > 1.0f) {
        t -= 1.0f;
    }

    if ((double)t < 0.16666666666666666) {
        return p + (q - p) * 6.0f * t;
    }

    if ((double)t < 0.5) {
        return q;
    }

    if ((double)t < 0.6666666666666666) {
        return p + (q - p) * (0.6666667f - t) * 6.0f;
    }
    return p;
}