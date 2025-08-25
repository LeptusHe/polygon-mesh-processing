#include "least-squares-vertex-baker.h"
#include "sampling/uniform-sampler.h"
#include "sampling/hammersley-sampler.h"
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

namespace meshlib {

LeastSquaresVertexBaker::LeastSquaresVertexBaker(Mesh& mesh, Texture& tex, const Options& options)
    : mesh_(mesh)
    , tex_(tex)
    , options_(options)
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

bool IsValidSample(const Eigen::Vector2f& sample)
{
    const auto x = sample[0];
    const auto y = sample[1];

    if (x < 0.0f || x > 1.0f || y < 0.0f || y > 1.0f)
        return false;

    const auto sum = x+ y;
    if (0.0f > sum || 1.0f < sum)
        return false;

    return true;
}


void LeastSquaresVertexBaker::Init()
{
    const auto sample_num = options_.sample_num;

    if (options_.enable_random_sample) {
        UniformSampler sampler;
        barycentric_coords_.resize(sample_num);

        for (int i = 0; i < sample_num; ++ i) {
            Eigen::Vector2f sample;
            do {
                sample = sampler.Get2D();
            } while (!IsValidSample(sample));

            barycentric_coords_[i] = BarycentricCoords(sample[0], sample[1], 1.0f - sample[0] - sample[1]);
        }
    } else {
        GenerateSamplingSamples(sample_num);
    }
}

void LeastSquaresVertexBaker::GenerateSamplingSamples(int sample_num)
{
    barycentric_coords_.resize(sample_num);

    for (int i = 0; i < sample_num; ++i)
    {
        double vdcVal = RadicalInverse((uint)i);
        auto uf = (uint32_t)(vdcVal * ((uint64_t)(1) << 32));
        Eigen::Vector2f A = Eigen::Vector2f(1, 0);
        Eigen::Vector2f B = Eigen::Vector2f(0, 1);
        Eigen::Vector2f C = Eigen::Vector2f(0, 0);
        for (int j = 0; j < 16; ++j)
        {
            uint32_t d = (uf >> (2 * (15 - j))) & (uint32_t)0x3;
            Eigen::Vector2f An = Eigen::Vector2f::Zero();
            Eigen::Vector2f Bn = Eigen::Vector2f::Zero();
            Eigen::Vector2f Cn = Eigen::Vector2f::Zero();
            switch (d)
            {
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
        barycentric_coords_[i] = BarycentricCoords(r.x(), r.y(), 1 - r.x() - r.y());
    }
}

void LeastSquaresVertexBaker::Solve()
{
    CalculateCoefficientMatrix();
    CalculateConstantVector();

    spdlog::stopwatch sw;
    SolveLinerEquation();
    spdlog::info("succeed to solve equation, time={}s", sw);
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
            if (!fh.is_valid())
                continue;

            e_prop_[eh] += 1.0f / 12.0f * mesh_.calc_face_area(fh);
        }
    }
}

void LeastSquaresVertexBaker::CalculateConstantVector()
{
    for (const auto vh : mesh_.vertices()) {
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        for (const auto fh : mesh_.vf_range(vh)) {
            sum += CalculateConstantFactor(fh, vh);
        }
        v_constant_prop_[vh] = sum;
    }

    const int sample_num = static_cast<int>(barycentric_coords_.size());
    for (const auto vh : mesh_.vertices()) {
        float sum_area = 0.0f;
        int sum_sample_num = 0;
        for (const auto fh : mesh_.vf_range(vh)) {
            sum_area += mesh_.calc_face_area(fh);
            sum_sample_num += sample_num;
        }
        v_constant_prop_[vh] *= sum_area / static_cast<float>(sum_sample_num);
    }
}

int get_triangle_index(int i)
{
    return i % 3;
}

Eigen::Vector3f LeastSquaresVertexBaker::CalculateConstantFactor(Mesh::FaceHandle fh, Mesh::VertexHandle v)
{
    Mesh::TexCoord2D coords[3];

    int v0_index = -1;
    int index = 0;
    for (const auto vh : mesh_.fv_range(fh)) {
        coords[index] = mesh_.texcoord2D(vh);
        if (vh == v) {
            v0_index = index;
        }
        index += 1;
    }

    if (options_.debug_integral_method) {
        v0_index = 0;
    }

    int v0_id = (v0_index + 0) % 3;
    int v1_id = (v0_index + 1) % 3;
    int v2_id = (v0_index + 2) % 3;

    const auto sample_num = barycentric_coords_.size();
    Eigen::Vector3f val = Eigen::Vector3f::Zero();
    for (int i = 0; i < sample_num; ++ i) {
        auto& barycentric_coord = barycentric_coords_[i];
        const auto uv = barycentric_coord.Interpolate(
            coords[v0_id],
            coords[v1_id],
            coords[v2_id]);

        // TODO: what is hat function?
        const auto flip_uv = OpenMesh::Vec2f(uv[0], 1 - uv[1]);
        val += barycentric_coord.x * tex_.Sample(flip_uv);
        //val += barycentric_coord.z * Eigen::Vector3f(1, 1, 1);
    }
    return val;
}


std::vector<Eigen::Triplet<float>> LeastSquaresVertexBaker::BuildEdgeRegularizationMatrix()
{
    std::vector<Eigen::Triplet<float>> triplets;
    for (const auto e : mesh_.edges()) {
        if (e.is_boundary())
            continue;

        const auto h0 = mesh_.halfedge_handle(e, 0);
        const auto h1 = mesh_.halfedge_handle(e, 1);

        const auto face0 = mesh_.face_handle(h0);
        const auto face1 = mesh_.face_handle(h1);

        VertexInfo face0_vertex_infos[3];
        float face0_area;
        ComputeVertexGradientInTriangle(face0, face0_area, face0_vertex_infos);

        VertexInfo face1_vertex_infos[3];
        float face1_area;
        ComputeVertexGradientInTriangle(face1, face1_area, face1_vertex_infos);

        std::map<OpenMesh::VertexHandle, Eigen::Vector3f> gradient_map;
        for (int i = 0; i < 3; ++ i) {
            const auto vh = face0_vertex_infos[i].vh;
            gradient_map[vh] = face0_vertex_infos[i].gradient;
        }

        for (int i = 0; i < 3; ++ i) {
            const auto vh = face1_vertex_infos[i].vh;
            if (gradient_map.find(vh) != std::end(gradient_map)) {
                gradient_map[vh] -= face1_vertex_infos[i].gradient;
            } else {
                gradient_map[vh] = -face1_vertex_infos[i].gradient;
            }
        }

        if (gradient_map.size() != 4) {
            throw std::runtime_error("gradient map size is not 4");
        }

        OpenMesh::VertexHandle vhs[4];
        int index = 0;
        for (const auto& [vh, gradient] : gradient_map) {
            vhs[index] = vh;
            index += 1;
        }

        float area = face0_area + face1_area;
        for (int i = 0; i < 4; ++ i) {
            for (int j = 0; j < 4; ++ j) {
                const auto lhs_vh = vhs[i];
                const auto rhs_vh = vhs[j];

                const auto factor = area * options_.regularization_factor;
                const auto val = factor * gradient_map[lhs_vh].dot(gradient_map[rhs_vh]);
                triplets.emplace_back(lhs_vh.idx(), rhs_vh.idx(), val);
            }
        }
    }
    return triplets;
}

void LeastSquaresVertexBaker::ComputeVertexGradientInTriangle(const OpenMesh::FaceHandle& fh, float& triangle_area, VertexInfo *vertex_infos)
{
    int vertex_index = 0;
    for (const auto vh : mesh_.fv_range(fh)) {
        const auto p = mesh_.point(vh);
        const auto pos = Eigen::Vector3f(p[0], p[1], p[2]);

        vertex_infos[vertex_index].vh = vh;
        vertex_infos[vertex_index].pos = pos;
        vertex_index += 1;
    }

    const auto v1_sub_v0 = vertex_infos[1].pos - vertex_infos[0].pos;
    const auto v2_sub_v0 = vertex_infos[2].pos - vertex_infos[0].pos;
    const auto normal_vec =  v1_sub_v0.cross(v2_sub_v0);
    const auto normalized_normal = normal_vec.normalized();
    triangle_area = 0.5f * normal_vec.norm();

    for (int i = 0; i < 3; ++ i) {
        const auto v0_idx = (i + 0) % 3;
        const auto v1_idx = (i + 1) % 3;
        const auto v2_idx = (i + 2) % 3;

        const auto edge_vec = vertex_infos[v2_idx].pos - vertex_infos[v1_idx].pos;
        auto gradient_v0 = normal_vec.cross(edge_vec);
        gradient_v0 *= 1.0f / (4.0f * triangle_area * triangle_area);
        gradient_v0 *= sqrt(triangle_area);
        
        vertex_infos[v0_idx].gradient = gradient_v0;
    }
}

void LeastSquaresVertexBaker::SolveLinerEquation()
{
    Eigen::SparseMatrix<float> A(mesh_.n_vertices(), mesh_.n_vertices());

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

    if (options_.enable_edge_regularization) {
        auto regularization_triples = BuildEdgeRegularizationMatrix();
        triples.insert(std::end(triples),
            std::begin(regularization_triples),
            std::end(regularization_triples));
    }

    A.setFromTriplets(triples.begin(), triples.end());

#if DEBUG_MATRIX
    Eigen::IOFormat fmt(6, 0, ", ", "\n", "[", "]");
    Eigen::MatrixXf mat = Eigen::MatrixXf(A);
    std::cout << mat.format(fmt) << std::endl;
#endif

    Eigen::SimplicialLLT<Eigen::SparseMatrix<float>> solver;
    //Eigen::SparseQR<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(A);

    Eigen::VectorXf x[3];
    for (int channel_idx = 0; channel_idx < 3; ++ channel_idx) {
        Eigen::VectorXf b = Eigen::VectorXf::Zero(mesh_.n_vertices());
        for (const auto vh : mesh_.vertices()) {
            b[vh.idx()] = v_constant_prop_[vh][channel_idx];
        }

        /*
        b[0] = 1.0 / 6 * 2;
        b[1] = 1 / 6.0;
        b[2] = 1 / 6.0;
        b[3] = 1 / 6.0 * 2.0;
         */

#if DEBUG_MATRIX
        std::cout << b.format(fmt) << std::endl;
#endif

        x[channel_idx] = solver.solve(b);

        if (solver.info() != Eigen::Success) {
            spdlog::error("failed to solve linear equation");
        } else {
            spdlog::info("succeed to solve linear equation");
        }

    }

    for (const auto vh : mesh_.vertices()) {
        float r = ConvertToColorValue(x[0][vh.idx()]);
        float g = ConvertToColorValue(x[1][vh.idx()]);
        float b = ConvertToColorValue(x[2][vh.idx()]);

#if DEBUG_MATRIX
        spdlog::info("c {}, {}, {}\n", r, g, b);
#endif

        mesh_.set_color(vh, Mesh::Color(r * 255.0f, g * 255.0f, b * 255.0f));
    }
}

float LeastSquaresVertexBaker::ConvertToColorValue(float v)
{
    if (std::isinf(v) || std::isnan(v)) {
        spdlog::error("invalid color: {}", v);
    }

    if (v < 0.0 || v > 1.0) {
        spdlog::error("invalid color: {}", v);
    }

    v = std::clamp(v, 0.0f, 1.0f);
    return v;
}

} // namespace meshlib