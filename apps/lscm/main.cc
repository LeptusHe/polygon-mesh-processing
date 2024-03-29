#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <igl/boundary_loop.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/lscm.h>
#include <fmt/format.h>
#include <unordered_set>
#include <Eigen/SparseQR>
#include <exception>
#include <filesystem>

Eigen::MatrixXd V;
Eigen::MatrixXi F;
Eigen::MatrixXd U;

constexpr double pi = 3.14159265358979323846;

namespace fs = std::filesystem;

std::vector<std::string> FindModelFilesInDir(const std::string& dir)
{
    std::vector<std::string> filepaths;

    for (const auto& entry : fs::directory_iterator(dir)) {
        const auto& filepath = entry.path();

        filepaths.push_back(filepath.string());
        std::cout << fmt::format("find file in dir: {}\n", filepath.string());
    }
    return filepaths;
}


Eigen::Vector2d ConvertToUV(double len)
{
    const double segment = 1.0 / 4.0;

    int x = static_cast<int>(std::floor(len / segment));
    double y = std::fmod(len, segment) / segment;

    switch (x) {
        case 0: return {0, y};
        case 1: return {y, 1};
        case 2: return {1, 1 - y};
        case 3: return {1 - y, 0};
        default: {
            auto msg = fmt::format("invalid coordinate: {}, {}, len={}", x, y, len);
            throw new std::exception(msg.c_str());
        }
    }
}

Eigen::Vector2d ConvertToCircleUV(double s)
{
    double theta = s * (2.0 * pi);
    double u = std::sin(theta);
    double v = std::cos(theta);
    return {u, v};
}

Eigen::MatrixXd FixBoundary(const Eigen::MatrixXd& V, const std::vector<int>& boundaryVertList)
{
    Eigen::MatrixXd boundaryUV;
    boundaryUV.resize(boundaryVertList.size(), 2);

    for (int i = 0; i < boundaryVertList.size(); ++ i) {
        double val = static_cast<double>(i) / static_cast<double>(boundaryVertList.size());
        //auto uv = ConvertToUV(val);
        auto uv = ConvertToCircleUV(val);
        //std::cout << fmt::format("({}, {})", uv.x(), uv.y()) << std::endl;
        boundaryUV.row(i) = uv;
    }
    return boundaryUV;
}

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
    switch(key)
    {
        case '1':
            viewer.data().set_vertices(V);
            viewer.core().align_camera_center(V, F);
            return true;
        case '2':
            viewer.data().set_vertices(U);
            viewer.core().align_camera_center(U, F);
            return true;
        default: break;
    }
    return false;
}

using TexMat = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>;

std::vector<TexMat> GenerateGridTex(Eigen::Vector3d colorA, Eigen::Vector3d colorB)
{
    unsigned size = 128;
    unsigned halfSize = size / 2;

    std::vector<TexMat> texList {
        TexMat::Zero(size, size),
        TexMat::Zero(size, size),
        TexMat::Zero(size, size),
    };

    for (int channel = 0; channel < 3; ++ channel) {
        auto& tex = texList[channel];

        for (unsigned i = 0; i < size; ++i) {
            for (unsigned j = 0; j < size; ++j) {
                tex(i, j) = std::clamp(static_cast<int>(colorA[channel] * 255), 0, 255);
                if ((i < halfSize && j < halfSize) || (i >= halfSize && j >= halfSize))
                    tex(i, j) = std::clamp(static_cast<int>(colorB[channel] * 255), 0, 255);
            }
        }
    }

    return texList;

    //texture_G = texture_R;
    //texture_B = texture_R;
    //texture_A = Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>::Constant(texture_R.rows(),texture_R.cols(),255);

    //viewer.data().set_texture()
}


int main(int argc, char *argv[])
{
    auto path = argc > 1 ? argv[1] : "data/beetle.obj";
    if (!igl::read_triangle_mesh(path, V, F)) {
        std::cout << fmt::format("failed to load mesh: [{0}]", path);
        return -1;
    } else {
        std::cout << fmt::format("vertex count: {}\n", V.rows());
        std::cout << fmt::format("face count: {}\n", F.rows());
    }

#if ENABLE_DEBUG
    std::cout << "vertex: " << std::endl;
    std::cout << V << std::endl;

    std::cout << "faces:" << std::endl;
    std::cout << F << std::endl;
#endif

    Eigen::VectorXi bnd, b(2, 1);
    igl::boundary_loop(F, bnd);

    b(0) = bnd(0);
    b(1) = bnd(bnd.size() / 2);

    Eigen::MatrixXd bc(2, 2);
    bc << 0, 0,
          1, 0;

    igl::lscm(V, F, b, bc, U);


    Eigen::MatrixXd scaleUV = 10 * U;
    auto texList = GenerateGridTex(Eigen::Vector3d{1, 1, 1}, Eigen::Vector3d{0.5, 0.0, 0.0});

    igl::opengl::glfw::Viewer viewer;
    auto modelId = viewer.data_list[0].id;
    viewer.data(modelId).set_mesh(V, F);
    viewer.data(modelId).set_colors(U);
    viewer.data(modelId).set_uv(scaleUV);
    viewer.data(modelId).show_texture = true;
    viewer.data(modelId).uniform_colors(Eigen::Vector3d {1, 1, 1}, Eigen::Vector3d {1, 1, 1}, Eigen::Vector3d{0, 0, 0});
    viewer.data(modelId).set_texture(texList[0], texList[1], texList[2]);

    int index = viewer.append_mesh();
    viewer.data(index).set_mesh(U, F);
    viewer.data(index).set_colors(U);
    viewer.data(index).set_uv(scaleUV);
    viewer.data(index).show_texture = true;
    viewer.data(index).uniform_colors(Eigen::Vector3d {1, 1, 1}, Eigen::Vector3d {1, 1, 1}, Eigen::Vector3d{0, 0, 0});
    viewer.data(index).set_texture(texList[0], texList[1], texList[2]);

    int w = 1080, h = 960;
    int left_view, right_view;

    viewer.callback_init = [&](igl::opengl::glfw::Viewer &v) -> bool {
        v.core().viewport = Eigen::Vector4f(0, 0, w / 2, h);
        left_view = v.core_list[0].id;

        auto viewport = Eigen::Vector4f(w / 2, 0, w - (w / 2), h);
        right_view = viewer.append_core(viewport);

        v.data(modelId).set_visible(true, left_view);
        v.data(modelId).set_visible(false, right_view);

        v.data(index).set_visible(false, left_view);
        v.data(index).set_visible(true, right_view);
        return false;
    };


    viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h) {
        v.core(left_view).viewport = Eigen::Vector4f(0, 0, w / 2, h);
        v.core(right_view).viewport = Eigen::Vector4f(w / 2, 0, w - (w / 2), h);
        return true;
    };

    viewer.callback_key_down = &key_down;

    viewer.launch(false, false, "viewer", w, h);

    return 0;
}