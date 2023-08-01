#include <iostream>
#include <filesystem>
#include <fmt/format.h>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <queue>
#include <utility>
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>
#include <OpenMesh/Tools/Decimater/ModNormalDeviationT.hh>
#include <OpenMesh/Tools/Decimater/ModNormalFlippingT.hh>
#include <OpenMesh/Tools/Decimater/MixedDecimaterT.hh>
#include <imgui.h>
#include <cmath>

#include "utils/vertex_merger.h"
#include "utils/mesh_utils.h"

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;
using Item = std::pair<double, Mesh::EdgeHandle>;
using PriorityQueue = std::priority_queue<Item, std::vector<Item>, std::greater<>>;

using Decimater = OpenMesh::Decimater::DecimaterT<Mesh>;
using HModQuadric = OpenMesh::Decimater::ModQuadricT<Mesh>::Handle;

bool ValidToCollapse0(const Mesh& mesh, Mesh::EdgeHandle eh)
{
    for (int i = 0; i < 2; ++ i) {
        auto halfEdgeHandle = mesh.halfedge_handle(eh, i);
        auto fh = mesh.face_handle(halfEdgeHandle);
        if (!fh.is_valid()) {
            continue;
        }

        if (mesh.is_boundary(fh)) {
            return false;
        }
    }
    return true;
}

bool ValidToCollapse(const Mesh& mesh, Mesh::EdgeHandle eh)
{
    auto halfEdgeHandle = mesh.halfedge_handle(eh, 0);
    auto v0 = mesh.to_vertex_handle(halfEdgeHandle);
    auto v1 = mesh.from_vertex_handle(halfEdgeHandle);

    if (mesh.is_boundary(v0) || mesh.is_boundary(v1)) {
        return false;
    }
    return true;
}

Mesh::VertexHandle CollapseEdge(Mesh& mesh, PriorityQueue& queue)
{
    Mesh::EdgeHandle edgeHandle;
    Mesh::HalfedgeHandle halfEdgeHandle;

    while (true) {
        if (queue.empty()) {
            return Mesh::VertexHandle();
        }

        edgeHandle = queue.top().second;
        queue.pop();

        if (!edgeHandle.is_valid() || mesh.status(edgeHandle).deleted()) {
            //std::cout << "invalid edge" << std::endl;
            //return Mesh::InvalidVertexHandle;
            continue;
        }

        halfEdgeHandle = mesh.halfedge_handle(edgeHandle, 0);
        if (!mesh.is_collapse_ok(halfEdgeHandle)) {
            //std::cout << "invalid to collapse" << std::endl;
            //return Mesh::InvalidVertexHandle;
            continue;
        }

        if (!ValidToCollapse(mesh, edgeHandle)) {
            std::cout << "invalid to collapse" << std::endl;
            continue;
        }

        break;
    }


    auto vertHandle0 = mesh.from_vertex_handle(halfEdgeHandle);
    auto vertHandle1 = mesh.to_vertex_handle(halfEdgeHandle);

    auto vert0 = mesh.point(vertHandle0);
    auto vert1 = mesh.point(vertHandle1);

    auto newVert = (vert0 + vert1) / 2.0;

    mesh.set_point(vertHandle1, newVert);
    mesh.collapse(halfEdgeHandle);
    mesh.garbage_collection();

    for (auto iter = mesh.cve_iter(vertHandle1); iter.is_valid(); ++ iter) {
        if (!ValidToCollapse(mesh, *iter)) {
            continue;
        }

        auto len = mesh.calc_edge_length(*iter);
        queue.emplace(len, iter);
    }
    return vertHandle1;

    /*
    for (auto vvIter = mesh.vv_iter(vertHandle1); vvIter.is_valid(); ++vvIter) {

        auto edgeHandle = mesh.edge_handle(*vvIter, vertHandle0);
        if (mesh.status(edgeHandle).deleted()) {
            continue;
        }

        auto vertHandle2 = mesh.to_vertex_handle(mesh.halfedge_handle(edgeHandle, 0));
        auto vertHandle3 = mesh.to_vertex_handle(mesh.halfedge_handle(edgeHandle, 1));

        auto vert2 = mesh.point(vertHandle2);
        auto vert3 = mesh.point(vertHandle3);

        auto cost = (vert2 - vert3).squaredNorm();

        queue.push(std::make_pair(cost, edgeHandle));
    }
     */
}


void PrintMeshInfo(const Mesh& mesh, const std::string& header = "")
{
    if (!header.empty()) {
        std::cout << header << std::endl;
    }

    std::cout << fmt::format("vertex count: {}\n", mesh.n_vertices());
    std::cout << fmt::format("face count: {}\n", mesh.n_faces());
    std::cout << fmt::format("edge count: {}\n", mesh.n_edges());
    std::cout << fmt::format("half edge count: {}\n\n", mesh.n_halfedges());
}

bool Collinear(Mesh& mesh, Mesh::HalfedgeHandle cur, Mesh::HalfedgeHandle next, float& cos_theta)
{
    auto v0 = mesh.from_vertex_handle(cur);
    auto v1 = mesh.to_vertex_handle(cur);
    auto v2 = mesh.to_vertex_handle(next);

    auto p0 = mesh.point(v0);
    auto p1 = mesh.point(v1);
    auto p2 = mesh.point(v2);

    auto l1 = (p0 - p1).normalized();
    auto l2 = (p2 - p1).normalized();

    cos_theta = l1.dot(l2);
    return std::abs(cos_theta + 1) < 1e-3;
}

void ColorBoundaryVertex(Mesh& mesh)
{
    for (auto vh : mesh.vertices()) {
        if (vh.is_boundary()) {
            mesh.set_color(vh, {1, 0, 0});
        } else {
            mesh.set_color(vh, {1, 1, 1});
        }
    }
}

void FindBoundaryEdge(Mesh& mesh, igl::opengl::glfw::Viewer& viewer)
{
    Mesh::HalfedgeHandle cur, next;
    bool found = false;

    for (auto hf : mesh.halfedges()) {
        cur = hf;

        if (!mesh.is_boundary(cur))
            continue;

        next = mesh.next_halfedge_handle(cur);
        if (!next.is_valid() || !mesh.is_boundary(next))
            continue;

        found = true;
        break;
    }

    float cos_theta = 0;
    Collinear(mesh, cur, next, cos_theta);
    std::cout << "cos theta: " << cos_theta << std::endl;

    if (found) {
        auto v0 = mesh.from_vertex_handle(cur);
        auto v1 = mesh.to_vertex_handle(cur);
        auto v2 = mesh.to_vertex_handle(next);

        auto p0 = mesh.point(v0);
        auto p1 = mesh.point(v1);
        auto p2 = mesh.point(v2);

        Eigen::Matrix3d p;
        p.row(0) = Eigen::Vector3d(p0[0], p0[1], p0[2]);
        p.row(1) = Eigen::Vector3d(p1[0], p1[1], p1[2]);
        p.row(2) = Eigen::Vector3d(p2[0], p2[1], p2[2]);

        auto strs = std::vector<std::string> {
            "v0",
            "v1",
            "v2"
        };
        viewer.data().show_custom_labels = true;
        viewer.data().label_size = 4;
        viewer.data().set_labels(p, strs);
    }
}

void FindCollinearBoundaryEdge(Mesh& mesh, igl::opengl::glfw::Viewer& viewer)
{
    Mesh::HalfedgeHandle cur, next;
    bool found = false;

    for (auto hf : mesh.halfedges()) {
        cur = hf;

        if (!mesh.is_boundary(cur))
            continue;

        next = mesh.next_halfedge_handle(cur);
        if (!next.is_valid() || !mesh.is_boundary(next))
            continue;

        float cos_theta = 0;
        if (!Collinear(mesh, cur, next, cos_theta))
            continue;

        if (!mesh.is_collapse_ok(next))
            continue;

        found = true;
        break;
    }

    if (found) {
        auto v0 = mesh.from_vertex_handle(cur);
        auto v1 = mesh.to_vertex_handle(cur);
        auto v2 = mesh.to_vertex_handle(next);

        auto p0 = mesh.point(v0);
        auto p1 = mesh.point(v1);
        auto p2 = mesh.point(v2);

        Eigen::Matrix3d p;
        p.row(0) = Eigen::Vector3d(p0[0], p0[1], p0[2]);
        p.row(1) = Eigen::Vector3d(p1[0], p1[1], p1[2]);
        p.row(2) = Eigen::Vector3d(p2[0], p2[1], p2[2]);

        auto strs = std::vector<std::string> {
            "v0",
            "v1",
            "v2"
        };
        viewer.data().show_custom_labels = true;
        viewer.data().label_size = 2;
        viewer.data().set_labels(p, strs);
    }
}

void CollapseBoundaryEdge(Mesh& mesh)
{
    Mesh::HalfedgeHandle cur, next;
    bool found = false;

    for (auto hf : mesh.halfedges()) {
        cur = hf;

        if (!mesh.is_boundary(cur))
            continue;

        next = mesh.next_halfedge_handle(cur);
        if (!next.is_valid() || !mesh.is_boundary(next))
            continue;

        float cos_theta = 0;
        if (!Collinear(mesh, cur, next, cos_theta))
            continue;

        if (!mesh.is_collapse_ok(next))
            continue;

        found = true;
        break;
    }

    if (found) {
        //auto v1 = mesh.to_vertex_handle(cur);
        //auto v2 = mesh.to_vertex_handle(next);

        //auto p2 = mesh.point(v2);
        //mesh.set_point(v1, p2);

        auto next_op = mesh.opposite_halfedge_handle(next);
        auto fh = mesh.face_handle(next_op);
        //mesh.delete_face(fh);

        if (mesh.is_collapse_ok(next)) {
            mesh.collapse(next);
        } else {
            std::cout << "failed to collapse half edge" << std::endl;
        }

        mesh.garbage_collection();
    } else {
        std::cout << "failed to find" << std::endl;
    }
}

int main(int argc, char *argv[])
{
    auto path = argc > 1 ? argv[1] : "data/beetle.obj";

    Mesh mesh;
    OpenMesh::IO::Options opt;
    if (!OpenMesh::IO::read_mesh(mesh, path, opt)) {
        std::cerr << "failed to load mesh from " << path << std::endl;
        return 1;
    } else {
        std::cout << fmt::format("succeed to load mesh from [{0}]", path) << std::endl;
        PrintMeshInfo(mesh);
    }

    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);
    Eigen::MatrixXi F = Eigen::MatrixXi::Zero(mesh.n_faces(), 3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(mesh.n_faces(), 3);

    for (auto fh : mesh.faces()) {
        if (mesh.is_boundary(fh)) {
            C.row(fh.idx()) = Eigen::Vector3d(1, 0, 0);
        } else {
            C.row(fh.idx()) = Eigen::Vector3d(0.5, 0.5f, 0.5f);
        }
    }

    PriorityQueue queue;
    for (auto edgeHandle : mesh.edges()) {
        if (!ValidToCollapse(mesh, edgeHandle)) {
            continue;
        }

        auto length = mesh.calc_edge_length(edgeHandle);
        queue.push(std::make_pair(length, edgeHandle));
    }


    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&plugin);
    plugin.widgets.push_back(&menu);

    int faceId = 0;
    int faceCount = 1;
    bool continueToCollapse = false;
    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        ImGui::InputInt("Face Id", &faceId);
        if (ImGui::Button("Collapse")) {
            auto fh = mesh.face_handle(faceId);
            auto halfedge = mesh.halfedge_handle(fh);
            if (mesh.is_collapse_ok(halfedge)) {
                mesh.collapse(halfedge);
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            }
        }

        if (ImGui::Button("Merge Vertex")) {
            vertex_merger vertexMerger;
            std::cout << "n_face: " << mesh.n_faces() << " vertex: " << mesh.n_vertices() << std::endl;
            mesh = vertexMerger.Merge(mesh);
            std::cout << "n_face: " << mesh.n_faces() << " vertex: " << mesh.n_vertices() << std::endl;
        }

        if (ImGui::Button("Color Boundary Vertex")) {
            mesh.request_vertex_colors();
            ColorBoundaryVertex(mesh);
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }

        if (ImGui::Button("Find Boundary Edge")) {
            FindBoundaryEdge(mesh, viewer);
        }

        if (ImGui::Button("Find Collinear Boundary Edge")) {
            FindCollinearBoundaryEdge(mesh, viewer);
        }

        if (ImGui::Button("Collapse Boundary")) {
            CollapseBoundaryEdge(mesh);
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }

        ImGui::Checkbox("Continue To Collapse", &continueToCollapse);

        ImGui::InputInt("Face Count", &faceCount);

        if (ImGui::Button("Collapse Edge")) {
            Mesh::VertexHandle vh;
            for (int i = 0; i < faceCount; ++ i) {
                vh = CollapseEdge(mesh, queue);
            }
            if (vh.is_valid()) {
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);

                auto item = queue.top();
                if (!item.second.is_valid()) {
                    return;
                }

                C = Eigen::VectorXd::Zero(mesh.n_vertices(), 3);
                for (auto vertexHandle : mesh.vertices()) {
                    if (vertexHandle != vh) {
                        C.row(vertexHandle.idx()) = Eigen::Vector3d(0.2f, 0.2f, 0.2f);
                    } else {
                        std::cout << "set color to red" << std::endl;
                        C.row(vertexHandle.idx()) = Eigen::Vector3d(1, 0, 0);
                    }
                }
                std::cout << "face cnt: " << mesh.n_faces() << std::endl;
                viewer.data().set_colors(C);
            }
        }
    };

    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer& viewer) {
        if (continueToCollapse && mesh.n_faces() > 10) {
            CollapseEdge(mesh, queue);
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }
        return false;
    };


    for (auto he : mesh.halfedges()) {
        if (!he.is_boundary())
            continue;

        auto vf = mesh.from_vertex_handle(he);
        auto vt = mesh.to_vertex_handle(he);

        mesh.status(vf).set_locked(true);
        mesh.status(vt).set_locked(true);
    }

    Decimater decimater(mesh);
    OpenMesh::Decimater::ModNormalFlippingT<Mesh>::Handle modNormalFlipping;

    HModQuadric modQuadric;
    decimater.add(modQuadric);
    decimater.add(modNormalFlipping);
    decimater.module(modNormalFlipping).set_max_normal_deviation(5.0f);

    decimater.initialize();
    int count = decimater.decimate(10000);
    std::cout << "decimate count: " << count << std::endl;

    mesh.garbage_collection();

    meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);

    viewer.data().set_colors(C);
    viewer.launch();

    return 0;
}