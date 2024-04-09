#include "vertex-baker.h"
#include "sampling/uniform-sampler.h"

namespace meshlib {

VertexBaker::VertexBaker(Mesh& mesh)
    : mesh_(mesh), v_prop_(mesh, "v_prop"), e_prop_(mesh, "e_prop") {
    for (const auto vh: mesh_.vertices()) {
        v_prop_[vh] = 0.0f;
    }

    for (const auto eh: mesh_.edges()) {
        e_prop_[eh] = 0.0f;
    }

    Init();
}


void VertexBaker::Init()
{
    const int sample_num = 256;
    barycentric_coords_.resize(sample_num);

    UniformSampler sampler;
    for (int i = 0; i < sample_num; ++ i) {
        const auto sample = sampler.Get2D();
        barycentric_coords_[i] = BarycentricCoords(sample[0], sample[1], 1.0f - sample[0] - sample[1]);
    }
}


void VertexBaker::Solve()
{
    CalculateCoefficientMatrix();
}


void VertexBaker::CalculateCoefficientMatrix()
{
    for (const auto fh: mesh_.faces()) {
        const auto area = mesh_.calc_face_area(fh);
        for (const auto vh: mesh_.fv_range(fh)) {
            v_prop_[vh] += 1.0f / 6.0f * area;
        }
    }

    for (const auto eh: mesh_.edges()) {
        for (int i = 0; i < 2; ++i) {
            const auto he = eh.h(i);
            if (!he.is_valid())
                continue;

            const auto fh = he.face();
            e_prop_[eh] += 1.0f / 12.0f * mesh_.calc_face_area(fh);
        }
    }
}

void VertexBaker::CalculateConstantVector()
{
    const auto sample_num = barycentric_coords_.size();
    for (int i = 0; i < )
}

} // namespace meshlib