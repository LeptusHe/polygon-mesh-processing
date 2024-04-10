#include "least-squares-vertex-baker.h"
#include "sampling/uniform-sampler.h"

namespace meshlib {

LeastSquaresVertexBaker::LeastSquaresVertexBaker(Mesh& mesh, Texture& tex)
    : mesh_(mesh)
    , tex_(tex)
    , v_prop_(mesh, "v_prop")
    , e_prop_(mesh, "e_prop")
    , v_constant_prop_(mesh, "v_constant_prop")
{
    for (const auto vh: mesh_.vertices()) {
        v_prop_[vh] = 0.0f;
        v_constant_prop_[vh] = Eigen::Vector3f::Zero();
    }

    for (const auto eh: mesh_.edges()) {
        e_prop_[eh] = 0.0f;
    }

    Init();
}


void LeastSquaresVertexBaker::Init()
{
    const int sample_num = 256;
    barycentric_coords_.resize(sample_num);

    UniformSampler sampler;
    for (int i = 0; i < sample_num; ++ i) {
        const auto sample = sampler.Get2D();
        barycentric_coords_[i] = BarycentricCoords(sample[0], sample[1], 1.0f - sample[0] - sample[1]);
    }
}


void LeastSquaresVertexBaker::Solve()
{
    CalculateCoefficientMatrix();
    CalculateConstantVector();
}


void LeastSquaresVertexBaker::CalculateCoefficientMatrix()
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

void LeastSquaresVertexBaker::CalculateConstantVector()
{
    for (const auto vh : mesh_.vertices()) {
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        for (const auto fh : mesh_.vf_range(vh)) {
            sum += CalculateConstantFactor(fh);
        }
        v_constant_prop_[vh] = sum;
    }

    const int sample_num = static_cast<int>(barycentric_coords_.size());
    for (const auto vh : mesh_.vertices()) {
        float sum_area = 0.0f;
        for (const auto fh : mesh_.vf_range(vh)) {
            sum_area += mesh_.calc_face_area(fh);
        }
        v_constant_prop_[vh] *= sum_area / static_cast<float>(sample_num);
    }
}

Eigen::Vector3f LeastSquaresVertexBaker::CalculateConstantFactor(Mesh::FaceHandle fh)
{
    Mesh::TexCoord2D coords[3];

    int index = 0;
    for (const auto vh : mesh_.fv_range(fh)) {
        coords[index] = mesh_.texcoord2D(vh);
        index += 1;
    }

    const auto area = mesh_.calc_face_area(fh);
    const auto sample_num = barycentric_coords_.size();
    Eigen::Vector3f val = Eigen::Vector3f::Zero();
    for (int i = 0; i < sample_num; ++ i) {
        auto& barycentric_coord = barycentric_coords_[i];
        const auto uv = barycentric_coord.Interpolate(coords[0], coords[1], coords[2]);

        // TODO: what is hat function?
        val += barycentric_coord.x * tex_.Sample(uv);
    }
    return val;
}

void LeastSquaresVertexBaker::SolveLinerEquation()
{
    Eigen::SparseMatrix<float> A;

    std::vector<Eigen::Triplet<float>> triples;
    for (const auto vh : mesh_.vertices()) {
        triples.emplace_back(vh.idx(), vh.idx(), v_prop_[vh]);
    }

    for (const auto eh : mesh_.edges()) {
        const auto v0 = eh.v0();
        const auto v1 = eh.v1();

        const auto val = e_prop_[eh];
        triples.emplace_back(v0.idx(), v1.idx(), val);
        triples.emplace_back(v1.idx(), v0.idx(), val);
    }

    A.setFromTriplets(triples.begin(), triples.end());

    Eigen::VectorXf x[3];
    for (int channel_idx = 0; channel_idx < 3; ++ channel_idx) {
        Eigen::VectorXf b = Eigen::VectorXf::Zero(mesh_.n_vertices());
        for (const auto vh : mesh_.vertices()) {
            b[vh.idx()] = v_constant_prop_[vh][channel_idx];
        }

        Eigen::SparseQR<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>> solver;
        solver.compute(A);
        x[channel_idx] = solver.solve(b);
    }

    for (const auto vh : mesh_.vertices()) {
        float r = ConvertToColorValue(x[0][vh.idx()]);
        float g = ConvertToColorValue(x[1][vh.idx()]);
        float b = ConvertToColorValue(x[2][vh.idx()]);

        mesh_.set_color(vh, Mesh::Color(r * 255.0f, g * 255.0f, b * 255.0f));
    }
}

float LeastSquaresVertexBaker::ConvertToColorValue(float v)
{
    return v;
}

} // namespace meshlib