#include <catch.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "geometry/line.h"
#include "pmp/clip/uv-clipper.h"

namespace meshlib {

bool floatApproxEqual(float a, float b, float epsilon = 1e-6f)
{
    return std::fabs(a - b) < epsilon;
}

template <typename T>
bool ApproxEqual(const T& lhs, const T& rhs)
{
    for (int i = 0; i < lhs.size(); ++ i) {
        if (!floatApproxEqual(lhs[i], rhs[i]))
            return false;
    }
    return true;
}

bool HasVertex(const Mesh& mesh, const Mesh::Point& v_p)
{
    for (const auto vh : mesh.vertices()) {
        auto p = mesh.point(vh);
        if (ApproxEqual(p, v_p))
            return true;
    }
    return false;
}


bool HasTexCoord(const Mesh& mesh, const Mesh::TexCoord2D& uv)
{
    for (const auto vh : mesh.vertices()) {
        auto v_uv = mesh.texcoord2D(vh);
        if (ApproxEqual(uv, v_uv))
            return true;
    }
    return false;
}

Mesh::TexCoord2D GetClippedUV(Mesh::TexCoord2D uv)
{
    uv[0] = uv[0] - std::floor(uv[0]);
    uv[1] = uv[1] - std::floor(uv[1]);
    return uv;
}



TEST_CASE("uv-clip", "[uv-clip]")
{
    Mesh mesh;

    const auto p0 = Mesh::Point(0.5, 0, 0.5);
    const auto p1 = Mesh::Point(-0.5, 0, 0);
    const auto p2 = Mesh::Point(-0.5, 0, 1);
    const auto v0 = mesh.add_vertex(p0);
    const auto v1 = mesh.add_vertex(p1);
    const auto v2 = mesh.add_vertex(p2);

    mesh.request_vertex_texcoords2D();

    const auto uv0 = Mesh::TexCoord2D(0.5, 0.5);
    const auto uv1 = Mesh::TexCoord2D(-0.5, 0.0);
    const auto uv2 = Mesh::TexCoord2D(-0.5, 1.0);

    mesh.set_texcoord2D(v0, uv0);
    mesh.set_texcoord2D(v1, uv1);
    mesh.set_texcoord2D(v2, uv2);

    mesh.add_face(std::vector<Mesh::VertexHandle>{v0, v1, v2});

    UVClipper clipper;
    clipper.process(mesh);

    const auto& clipped_mesh = clipper.get_clipped_mesh();

#if ENABLE_LOG || _DEBUG
    for (const auto fh : clipped_mesh.faces()) {
        INFO("face info: \n");
        for (const auto vh: clipped_mesh.fv_range(fh)) {
            const auto uv = clipped_mesh.texcoord2D(vh);
            INFO("uv: " << uv[0] << ", " << uv[1] << "\n");
        }
    }
#endif


    SECTION("vertex and face count of clipped mesh") {
        REQUIRE(clipped_mesh.n_vertices() == 7);
        REQUIRE(clipped_mesh.n_faces() == 3);
    }

    SECTION("clip vertex data - position") {
        REQUIRE(HasVertex(clipped_mesh, Mesh::Point(0, 0, 0.25)));
        REQUIRE(HasVertex(clipped_mesh, Mesh::Point(0, 0, 0.25 + 0.5)));

        REQUIRE(HasVertex(clipped_mesh, p0));
        REQUIRE(HasVertex(clipped_mesh, p1));
        REQUIRE(HasVertex(clipped_mesh, p2));
    }

    SECTION("clip vertex data - uv") {
        REQUIRE(HasTexCoord(clipped_mesh, Mesh::TexCoord2D(0, 0.25)));
        REQUIRE(HasTexCoord(clipped_mesh, Mesh::TexCoord2D(0, 0.25 + 0.5)));

        REQUIRE(HasTexCoord(clipped_mesh, GetClippedUV(uv0)));
        REQUIRE(HasTexCoord(clipped_mesh, GetClippedUV(uv1)));
        REQUIRE(HasTexCoord(clipped_mesh, GetClippedUV(uv2)));
    }
}

} // namespace meshlib