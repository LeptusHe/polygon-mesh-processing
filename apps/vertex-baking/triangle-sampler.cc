#include "triangle-sampler.h"
#include "sampling/hammersley-sampler.h"

namespace meshlib {

Eigen::Vector3f TriangleSampler::GenerateSamples(int sample_num)
{
    std::vector<BarycentricCoords> barycentric_coords(sample_num);

    for (int i = 0; i < sample_num; ++i) {
        double vdcVal = RadicalInverse((uint)i);
        auto uf = (uint32_t)(vdcVal * ((uint64_t)(1) << 32));
        Eigen::Vector2f A = Eigen::Vector2f(1, 0);
        Eigen::Vector2f B = Eigen::Vector2f(0, 1);
        Eigen::Vector2f C = Eigen::Vector2f(0, 0);
        for (int j = 0; j < 16; ++j) {
            uint32_t d = (uf >> (2 * (15 - j))) & (uint32_t)0x3;
            Eigen::Vector2f An = Eigen::Vector2f::Zero();
            Eigen::Vector2f Bn = Eigen::Vector2f::Zero();
            Eigen::Vector2f Cn = Eigen::Vector2f::Zero();
            switch (d) {
            case 0:
                An = (B + C) / 2;
                Bn = (A + C) / 2;
                Cn = (A + B) / 2;
                break;
            case 1:
                An = A;
                Bn = (A + B) / 2;
                Cn = (A + C) / 2;
                break;
            case 2:
                An = (B + A) / 2;
                Bn = B;
                Cn = (B + C) / 2;
                break;
            case 3:
                An = (C + A) / 2;
                Bn = (C + B) / 2;
                Cn = C;
                break;
            }
            A = An;
            B = Bn;
            C = Cn;
        }
        const auto r = (A + B + C) / 3;
        barycentric_coords[i] = BarycentricCoords(r.x(), r.y(), 1 - r.x() - r.y());
    }

    return barycentric_coords;
}

} // namespace meshlib