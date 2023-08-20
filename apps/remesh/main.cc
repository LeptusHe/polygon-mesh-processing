#include <iostream>
#include <fmt/format.h>
#include <igl/opengl/glfw/Viewer.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <imgui.h>
#include "utils/mesh_utils.h"
#include "remesh/remesher.h"
#include "utils/vertex_merger.h"
#include "repair/repair.h"
#include "simplification/simplification.h"
#include <spdlog/spdlog.h>

#include <boost/bimap.hpp>
#include <boost/bimap/set_of.hpp>
#include <boost/bimap/multiset_of.hpp>

#include <CGAL/IO/polygon_soup_io.h>
#include <CGAL/polygon_mesh_processing/remesh.h>
#include <CGAL/polygon_mesh_processing/merge_border_vertices.h>
#include <CGAL/polygon_mesh_processing/remesh_planar_patches.h>
#include <CGAL/polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/surface_Delaunay_remeshing.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/region_growing.h>
#include <igl/file_dialog_open.h>

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;

#define ENABLE_DEBUG_LOG 1

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

void PrintMeshInfo(const CMesh& mesh, const std::string& header = "")
{
    if (!header.empty()) {
        std::cout << header << std::endl;
    }

    std::cout << fmt::format("vertex count: {}\n", mesh.num_vertices());
    std::cout << fmt::format("face count: {}\n", mesh.num_faces());
    std::cout << fmt::format("edge count: {}\n", mesh.num_edges());
    std::cout << fmt::format("half edge count: {}\n\n", mesh.num_halfedges());
}

bool SplitOneLongEdge(Mesh& mesh, float target_edge_length)
{
    for (auto edge: mesh.edges()) {
        if (!edge.is_valid()) {
            spdlog::error("invalid edge for collapse");
            continue;
        }

        if (mesh.calc_edge_length(edge) > target_edge_length) {
            auto mid_point = mesh.calc_edge_midpoint(edge);
            mesh.split(edge, mid_point);
            return true;
        }
    }
    return false;
}

void SplitLongEdges(Mesh& mesh, float target_edge_length)
{
    using Bimap = boost::bimap<
            boost::bimaps::set_of<OpenMesh::EdgeHandle>,
            boost::bimaps::multiset_of<float, std::greater<>>>;
    using long_edge = Bimap::value_type;

    float squared_len_high = target_edge_length * target_edge_length;
    Bimap long_edges;
    for (auto eh : mesh.edges()) {
        float squared_len = mesh.calc_edge_sqr_length(eh);
        if (squared_len > squared_len_high) {
            long_edges.insert(long_edge(eh, squared_len));
        }
    }

    while (!long_edges.empty()) {
        auto e_iter = long_edges.right.begin();
        auto eh = e_iter->second;
        long_edges.right.erase(e_iter);

        auto mid_point = mesh.calc_edge_midpoint(eh);
        mesh.split(eh, mid_point);
    }
    return;

    while (true) {
        if (!SplitOneLongEdge(mesh, target_edge_length)) {
            break;
        }
    }
}

bool CollapseOneShortEdge(Mesh& mesh, float short_len_threshold, float long_len_threshold)
{
    OpenMesh::EdgeHandle e;
    for (auto edge : mesh.edges()) {
        if (!edge.is_valid()) {
            spdlog::error("invalid edge for collapse");
            continue;
        }

        if (edge.is_boundary())
            continue;


        auto v0 = edge.vertex(0);
        auto v1 = edge.vertex(1);
        if (v0.is_boundary() || v1.is_boundary())
            continue;

        if (mesh.calc_edge_length(edge) < short_len_threshold) {
            e = edge;
            break;
        }
    }

    if (!e.is_valid()) {
        return false;
    }

    auto hf = mesh.halfedge_handle(e, 0);
    auto v0 = mesh.from_vertex_handle(hf);
    auto v1 = mesh.to_vertex_handle(hf);

    bool collapse_ok = true;
    auto p1 = mesh.point(v1);
    for (auto vh : mesh.vv_range(v0)) {
        auto p0 = mesh.point(vh);
        if ((p0 - p1).norm()  > long_len_threshold) {
            collapse_ok = false;
            break;
        }
    }

    if (collapse_ok && mesh.is_collapse_ok(hf)) {
        mesh.collapse(hf);
        return true;
    }
    return false;
}

void CollapseShortEdge(Mesh& mesh, float short_len_threshold, float long_len_threshold)
{
    while (true) {
        if (!CollapseOneShortEdge(mesh, short_len_threshold, long_len_threshold)) {
            break;
        }
    }
}

bool CollapseHalfEdge(Mesh& mesh, const OpenMesh::HalfedgeHandle& he, float long_edge_sqr_len)
{
    auto v0 = mesh.from_vertex_handle(he);
    auto v1 = mesh.to_vertex_handle(he);
    auto p1 = mesh.point(v1);

    bool collapse_ok = true;
    for (auto vh : mesh.voh_range(v0)) {
        auto sqr_dist = (p1 - mesh.point(mesh.to_vertex_handle(vh))).sqrnorm();

        if ((sqr_dist > long_edge_sqr_len)
            || (mesh.status(mesh.edge_handle(vh)).feature())
            || (mesh.is_boundary(mesh.edge_handle(vh)))) {
            collapse_ok = false;
            break;
        }
    }

    if (collapse_ok && mesh.is_collapse_ok(he)) {
        mesh.collapse(he);
        return true;
    }
    return false;
}

void CollapseShortEdges(Mesh& mesh, float short_edge_len, float long_edge_len)
{
    const float short_edge_sqr_len = short_edge_len * short_edge_len;
    const float long_edge_sqr_len = long_edge_len * long_edge_len;

    OpenMesh::EPropHandleT<bool> visited;
    auto visited_property_name = "visited-prop";
    if (!mesh.get_property_handle(visited, visited_property_name)) {
        mesh.add_property(visited, visited_property_name);
    }

    for (auto edge : mesh.edges()) {
        mesh.property(visited, edge) = false;
    }

    while (true) {
        bool generate_new_edge = false;

        for (auto iter = mesh.edges_begin(); iter != mesh.edges_end(); ++ iter) {
            if (mesh.property(visited, *iter))
                continue;

            mesh.property(visited, *iter) = true;
            auto sqr_len = mesh.calc_edge_sqr_length(*iter);

            if ((sqr_len >= short_edge_sqr_len) || (sqr_len <= std::numeric_limits<float>::epsilon()))
                continue;

            auto he = mesh.halfedge_handle(*iter, 0);
            auto oe = mesh.halfedge_handle(*iter, 1);

            generate_new_edge = CollapseHalfEdge(mesh, he, long_edge_sqr_len);
            if (!generate_new_edge) {
                generate_new_edge = CollapseHalfEdge(mesh, oe, long_edge_sqr_len);
            }
        }

        if (!generate_new_edge) {
            break;
        }
    }

    mesh.remove_property(visited);
    mesh.garbage_collection();
}

int GetTargetValence(const Mesh& mesh, OpenMesh::VertexHandle vh)
{
    return mesh.is_boundary(vh) ? 4 : 6;
}

void EqualizeValencesOfEdge(Mesh& mesh, OpenMesh::EdgeHandle edge)
{
    auto hf0 = mesh.halfedge_handle(edge, 0);
    auto hf1 = mesh.halfedge_handle(edge, 1);

    auto v0 = mesh.from_vertex_handle(hf0);
    auto v1 = mesh.to_vertex_handle(hf0);

    auto next_hf0 = mesh.next_halfedge_handle(hf0);
    auto v2 = mesh.to_vertex_handle(next_hf0);

    auto next_hf1 = mesh.next_halfedge_handle(hf1);
    auto v3 = mesh.to_vertex_handle(next_hf1);

    auto vertices = new OpenMesh::VertexHandle[4] { v0, v1, v2, v3 };

    int deviation_pre = 0;
    for (int i = 0; i < 4; i++) {
        auto vertex = vertices[i];
        int valence = static_cast<int>(mesh.valence(vertex));
        int target_valence = GetTargetValence(mesh, vertex);
        deviation_pre += std::abs(valence - target_valence);
    }

    if (!mesh.is_flip_ok(edge)) {
#if ENABLE_DEBUG_LOG
        std::cout << "failed to flip edge" << std::endl;
#endif
        return;
    }

    mesh.flip(edge);

    int deviation_post = 0;
    for (int i = 0; i < 4; i++) {
        auto vertex = vertices[i];
        int valence = static_cast<int>(mesh.valence(vertex));
        int target_valence = GetTargetValence(mesh, vertex);
        deviation_post += std::abs(valence - target_valence);
    }

    if (!edge.is_valid()) {
#if ENABLE_DEBUG_LOG
        std::cout << "failed to find halfedge" << std::endl;
#endif
        return;
    }

    // note: 相等时不要flip，很重要
    if (deviation_post >= deviation_pre) {
        if (!mesh.is_flip_ok(edge)) {

#if ENABLE_DEBUG_LOG
            std::cout << "failed to reflip edge" << std::endl;
#endif

            return;
        }

        mesh.flip(edge);
    }
}

void EqualizeValences(Mesh& mesh)
{
    for (int i = 0; i < mesh.n_edges(); ++ i) {
        auto edge = mesh.edge_handle(i);

        if (!edge.is_valid()) {
            spdlog::error("invalid edge for equalize");
            continue;
        }

        if (!mesh.is_flip_ok(edge) || mesh.status(edge).feature())
            continue;

#if ENABLE_DEBUG_LOG
        auto he = mesh.halfedge_handle(edge, 0);
        /*
        spdlog::info("equalize valences: face: {0}, edge: {1}, from: {2}, to: {3}",
                     mesh.face_handle(he).idx(),
                     edge.idx(),
                     mesh.from_vertex_handle(he).idx(),
                     mesh.to_vertex_handle(he).idx());
        */
#endif

        EqualizeValencesOfEdge(mesh, edge);
    }
}

void TangentialRelaxation(Mesh& mesh)
{
    OpenMesh::VPropHandleT<OpenMesh::Vec3f> point_prop;
    const auto point_prop_name = "relaxation-point";
    if (!mesh.get_property_handle(point_prop, point_prop_name)) {
        mesh.add_property(point_prop, point_prop_name);
    }

    for (auto vh : mesh.vertices()) {
        if (mesh.is_boundary(vh))
            continue;

        int n = 0;
        OpenMesh::Vec3f q(0, 0, 0);
        for (auto adj_vh : mesh.vv_range(vh)) {
            q += mesh.point(adj_vh);
            n += 1;
        }
        q = q / n;

        mesh.property(point_prop, vh) = q;
    }

    for (auto vh : mesh.vertices()) {
        // note: 是否要对边界顶点进行处理?
        if (mesh.is_boundary(vh))
            continue;

        auto q = mesh.property(point_prop, vh);
        auto p = mesh.point(vh);
        auto normal = mesh.calc_normal(vh);

        auto new_point = q + normal.dot(p - q) * normal;
        mesh.set_point(vh, new_point);
    }
}

void IncrementalRemesh(Mesh& mesh, OpenMesh::VProp<OpenMesh::Vec3f>& prop, float target_edge_len, int iterative_cnt)
{
    auto long_edge_len = 4.0f / 3.0f * target_edge_len;
    auto short_edge_len = 3.0f / 5.0f * target_edge_len;

    for (int i  = 0; i < iterative_cnt; ++ i) {
        SplitLongEdges(mesh, long_edge_len);
        mesh.garbage_collection();

        CollapseShortEdges(mesh, short_edge_len, long_edge_len);
        mesh.garbage_collection();

        EqualizeValences(mesh);
        mesh.garbage_collection();

        TangentialRelaxation(mesh);
        mesh.garbage_collection();
    }
}

bool ReadMesh(const std::string& path, Mesh& mesh, CMesh& cmesh)
{
    OpenMesh::IO::Options opt = OpenMesh::IO::Options::VertexNormal;
    if (!OpenMesh::IO::read_mesh(mesh, path, opt)) {
        std::cerr << "failed to load mesh from " << path << std::endl;
        return false;
    } else {
        std::cout << fmt::format("succeed to load mesh from [{0}]", path) << std::endl;
        PrintMeshInfo(mesh);
    }

    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();
    mesh.request_halfedge_normals();

    if (!mesh.has_vertex_normals()) {
        std::cerr << "the mesh has no vertex normals" << std::endl;
    }

    mesh.request_vertex_normals();
    mesh.request_face_normals();
    mesh.update_normals();

    if (!pmp::IO::read_polygon_mesh(path, cmesh)) {
        spdlog::error("failed to read mesh from path {}", path);
        return false;
    } else {
        spdlog::info("succeed to read cgal mesh from path {}", path);
        PrintMeshInfo(cmesh);
    }

    return true;
}

void WriteMesh(CMesh& mesh, const std::string& filepath)
{
    CGAL::IO::write_polygon_mesh(filepath, mesh, CGAL::parameters::stream_precision(17));
}


int main(int argc, char *argv[])
{
    auto path = argc > 1 ? argv[1] : "data/quad.obj";

    Mesh mesh;
    CMesh cmesh;
    ReadMesh(path, mesh, cmesh);


    auto prop_points = OpenMesh::VProp<OpenMesh::Vec3f>(mesh, "points");

    igl::opengl::glfw::Viewer viewer;
    viewer.data().label_size = 2.0f;
    viewer.data().face_based = true;

    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    igl::opengl::glfw::imgui::ImGuiMenu menu;

    viewer.plugins.push_back(&plugin);
    plugin.widgets.push_back(&menu);

    bool project = true;
    bool keep_boundary = true;
    float target_edge_length = 0.5f;
    int iterative_cnt = 10;
    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        if (ImGui::Button("load mesh")) {
            auto path = igl::file_dialog_open();
            ReadMesh(path, mesh, cmesh);
            prop_points = OpenMesh::VProp<OpenMesh::Vec3f>(mesh, "points");
        }

        if (ImGui::Button("merge vertex")) {
            vertex_merger vertexMerger;
            std::cout << "n_face: " << mesh.n_faces() << " vertex: " << mesh.n_vertices() << std::endl;
            //mesh = vertexMerger.Merge(mesh);
            std::cout << "n_face: " << mesh.n_faces() << " vertex: " << mesh.n_vertices() << std::endl;
        }

        ImGui::InputFloat("target edge length", &target_edge_length);
        auto short_edge_length = 4.0f / 5.0f * target_edge_length;
        auto long_edge_length = 4.0f / 3.0f * target_edge_length;

        ImGui::LabelText("short edge length", "%f", short_edge_length);
        ImGui::LabelText("long edge length", "%f", long_edge_length);

        if (ImGui::Button("Component")) {

        }

        if (ImGui::CollapsingHeader("Remesh Testing")) {
            ImGui::InputInt("iteration count: ", &iterative_cnt);
            auto proxy = mesh;
            if (ImGui::Button("Remesh")) {
                IncrementalRemesh(proxy, prop_points, target_edge_length, iterative_cnt);
                meshlib::MeshUtils::ConvertMeshToViewer(proxy, viewer);
            }

            if (ImGui::Button("remesher")) {
                meshlib::Remesher remesher;
                remesher.IsotropicRemeshing(proxy, target_edge_length, iterative_cnt);
                meshlib::MeshUtils::ConvertMeshToViewer(proxy, viewer);
            }

            if (ImGui::Button("Split One Long Edges")) {
                SplitOneLongEdge(mesh, long_edge_length);
                mesh.garbage_collection();
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            }

            if (ImGui::Button("Split Long Edges")) {
                SplitLongEdges(mesh, long_edge_length);
                mesh.garbage_collection();
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            }

            if (ImGui::Button("Collapse Short Edges")) {
                auto& cloneMesh = mesh;
                CollapseShortEdge(cloneMesh, short_edge_length, long_edge_length);
                cloneMesh.garbage_collection();
                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }

            if (ImGui::Button("openmesh collapse")) {
                auto& cloneMesh = mesh;
                CollapseShortEdges(cloneMesh, short_edge_length, long_edge_length);
                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }

            if (ImGui::Button("Equalize Valences")) {
                EqualizeValences(mesh);
                mesh.garbage_collection();
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            }

            if (ImGui::Button("Tangential Relaxation")) {
                TangentialRelaxation(mesh);
                mesh.garbage_collection();
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            }
        }

        ImGui::Spacing();
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("CGAL Algorithms")) {
            ImGui::Checkbox("enable project", &project);
            ImGui::Checkbox("keep boundary", &keep_boundary);

            if (ImGui::Button("cgal split long")) {
                auto &cloneMesh = cmesh;
                pmp::split_long_edges(cloneMesh.edges(), long_edge_length, cloneMesh);
                cloneMesh.collect_garbage();
                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }

            if (ImGui::Button("cgal collapse")) {
                auto& cloneMesh = cmesh;
                pmp::isotropic_remeshing(CGAL::faces(cloneMesh), static_cast<double>(target_edge_length), cloneMesh,
                                         pmp::parameters::number_of_iterations(1)
                                                 .do_flip(false)
                                                 .do_flip(false)
                                                 .number_of_relaxation_steps(0)
                                                 .do_project(false)
                                                 .protect_constraints(true));
                cloneMesh.collect_garbage();
                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }

            if (ImGui::Button("cgal tangential relaxation")) {
                auto &cloneMesh = cmesh;
                pmp::tangential_relaxation(cloneMesh);
                cloneMesh.collect_garbage();
                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }

            if (ImGui::Button("repair hole"))  {
                auto cloneMesh = cmesh;

                int hole_cnt = FillSmallHoles(cloneMesh, 5, 10);
                spdlog::info("hole count: {}", hole_cnt);

                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);
            }

            if (ImGui::Button("cgal remesh")) {
                auto cloneMesh = cmesh;

                int hole_cnt = 0;
                do {
                    cloneMesh = Repair(cloneMesh);
                    pmp::merge_duplicated_vertices_in_boundary_cycles(cloneMesh);
                    pmp::stitch_boundary_cycles(cloneMesh);
                    pmp::stitch_borders(cloneMesh);

                    pmp::isotropic_remeshing(CGAL::faces(cloneMesh), static_cast<double>(target_edge_length), cloneMesh,
                                            pmp::parameters::number_of_iterations(50)
                                                    .protect_constraints(keep_boundary)
                                                    .do_project(project));

                    hole_cnt = FillSmallHoles(cloneMesh, 5, 10);
                    spdlog::info("hole count: {}", hole_cnt);
                    cloneMesh.collect_garbage();
                } while (hole_cnt > 0);

                meshlib::MeshUtils::ConvertMeshToViewer(cloneMesh, viewer);

                cmesh = cloneMesh;
            }

            if (ImGui::Button("get components")) {
                using face_descriptor = boost::graph_traits<CMesh>::face_descriptor;

                auto f_map = cmesh.add_property_map<CMesh::Face_index, int>("f:component-id").first;

                auto num_comp = pmp::connected_components(cmesh, f_map);
                spdlog::info("component num: {}", num_comp);
            }

            if (ImGui::Button("cagl plannar patch remesh")) {
                //pmp::remesh
                auto cloneMesh = cmesh;

                //int invalid_edges_cnt = meshlib::CheckInvalidEdges(cloneMesh);
                //spdlog::info("invalid edges: {}", invalid_edges_cnt);

                auto f_map = cloneMesh.add_property_map<CMesh::Face_index, int>("f:component-id").first;
                auto num_comp = pmp::connected_components(cloneMesh, f_map);
                spdlog::info("before clone mesh component num: {}", num_comp);

                pmp::merge_duplicated_vertices_in_boundary_cycles(cloneMesh);

                std::vector<CMesh::Point> points;
                std::vector<std::vector<std::size_t>> polygon;
                pmp::polygon_mesh_to_polygon_soup(cloneMesh, points, polygon);


                int num_removed = pmp::merge_duplicate_points_in_polygon_soup(points, polygon);
                /*
                , [](const CMesh::Point& lhs, const CMesh::Point& rhs) {
                    constexpr float maxDigits = 100.0f;

                    auto lhs_x = static_cast<int64_t>(lhs[0] * maxDigits);
                    auto lhs_y = static_cast<int64_t>(lhs[1] * maxDigits);
                    auto lhs_z = static_cast<int64_t>(lhs[2] * maxDigits);

                    auto rhs_x = static_cast<int64_t>(rhs[0] * maxDigits);
                    auto rhs_y = static_cast<int64_t>(rhs[1] * maxDigits);
                    auto rhs_z = static_cast<int64_t>(rhs[2] * maxDigits);

                    return (lhs_x == rhs_x) && (lhs_y == rhs_y) && (lhs_z == rhs_z);
                });
                 */
                spdlog::info("removed vertex: {}", num_removed);

                num_removed = RemoveDuplicationVertex(points, polygon);
                spdlog::info("custom removed vertex: {}", num_removed);

                auto reverse_faces = FixInvalidOrientation(points, polygon);
                spdlog::info("reverse faces: {}", reverse_faces);

                auto sorted_points = SortPoints(points);
                PrintSortedPoints(sorted_points);

                auto num_faces = pmp::merge_duplicate_polygons_in_polygon_soup(points, polygon);
                spdlog::info("remove face: {}", num_faces);

                pmp::repair_polygon_soup(points, polygon);

                auto another_num_removed = pmp::merge_duplicate_points_in_polygon_soup(points, polygon);
                spdlog::info("remove again vertex: {}", another_num_removed);


                //auto stitch_cnt = pmp::stitch_borders(cloneMesh);
                //std::cout << "stitch count: " << stitch_cnt << std::endl;

                //pmp::remove_degenerate_edges()
                //pmp::remove_degenerate_faces()
                if (!pmp::orient_polygon_soup(points, polygon)) {
                    spdlog::info("invalid orient");
                }

                //reverse_faces = FixInvalidOrientation(points, polygon);
                //spdlog::info("reverse faces: {}", reverse_faces);

                CMesh clean_mesh;
                pmp::polygon_soup_to_polygon_mesh(points, polygon, clean_mesh);

                //invalid_edges_cnt = meshlib::CheckInvalidEdges(cloneMesh);
                //spdlog::info("invalid edges: {}", invalid_edges_cnt);

                //int hole_cnt = FillSmallHoles(clean_mesh, 5.0, 10);
                //spdlog::info("fill hole count: {}", hole_cnt);

                /*
                points.clear();
                polygon.clear();
                pmp::polygon_mesh_to_polygon_soup(clean_mesh, points, polygon);
                pmp::repair_polygon_soup(points, polygon);

                //reverse_faces = FixInvalidOrientation(points, polygon);
                //spdlog::info("fill hole count: {}", hole_cnt);

                pmp::polygon_soup_to_polygon_mesh(points, polygon, clean_mesh);
                //pmp::orient_polygon_soup()
                 */


                f_map = clean_mesh.add_property_map<CMesh::Face_index, int>("f:component-id").first;
                num_comp = pmp::connected_components(clean_mesh, f_map);
                spdlog::info("after clone mesh component num: {}", num_comp);

                int stitch_cnt = pmp::stitch_boundary_cycles(clean_mesh);
                pmp::remove_degenerate_faces(clean_mesh);
                pmp::orient(clean_mesh);
                pmp::merge_reversible_connected_components(clean_mesh);

                clean_mesh.collect_garbage();

                clean_mesh = Repair(clean_mesh);
                spdlog::info("stitch count: {}", stitch_cnt);


                //auto invalid_edges_cnt = meshlib::CheckInvalidEdges(clean_mesh);
                //spdlog::info(invalid_edges_cnt);


                //auto non_manifold_vertex_cnt = pmp::duplicate_non_manifold_vertices(clean_mesh);
                //spdlog::info("duplicate non manifold vertex: {}", non_manifold_vertex_cnt);

                CMesh outMesh;
                float cos_theta = 0.99;
                pmp::remesh_planar_patches(clean_mesh, outMesh, pmp::parameters::cosine_of_maximum_angle(cos_theta));
                                           //pmp::parameters::cosine_of_maximum_angle(0.99));
                //pmp::remesh_almost_planar_patches(cloneMesh, outMesh);
                outMesh.collect_garbage();
                //pmp::remove_almost_degenerate_faces(outMesh);

                f_map = outMesh.add_property_map<CMesh::Face_index, int>("f:component-id").first;
                num_comp = pmp::connected_components(outMesh, f_map);
                spdlog::info("remesh component num: {}", num_comp);

                meshlib::MeshUtils::ConvertMeshToViewer(outMesh, viewer);

                spdlog::info("old: vertex: {}, faces: {}", cmesh.num_vertices(), cmesh.num_faces());
                spdlog::info("new: vertex: {}, faces: {}", outMesh.num_vertices(), outMesh.num_faces());


                for (auto vertex : outMesh.vertices()) {
                    auto point = outMesh.point(vertex);
                    //spdlog::info("v: {}, {}, {}", point[0], point[1], point[2]);
                }

                WriteMesh(outMesh, "data/remesh-result.obj");
            }

            if (ImGui::Button("Continue To Collapse Boundary")) {
                int count = 0;
                bool found = true;
                do {
                    found = meshlib::CollapseBoundaryEdge(mesh);
                    count += 1;
                } while (found);

                std::cout << "found: " << count << std::endl;
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
            }
        }

        if (ImGui::Button("remesh almost planar patch")) {
            CMesh out_mesh;

            //pmp::region_growing_of_planes_on_faces(cmesh, )
            //pmp::remesh_almost_planar_patches(cmesh, out_mesh, )
        }

        if (ImGui::Button("delaunay remesh")) {
            using EIFMap = boost::property_map<CMesh, CGAL::edge_is_feature_t>::type;

            EIFMap eif = CGAL::get(CGAL::edge_is_feature, cmesh);
            pmp::detect_sharp_edges(cmesh, 45, eif);

            auto out_mesh = pmp::surface_Delaunay_remeshing(cmesh,
                                                            CGAL::parameters::protect_constraints(false).
                                                            mesh_edge_size(10.0).
                                                            mesh_facet_distance(10.0).
                                                            edge_is_constrained_map(eif));

            meshlib::MeshUtils::ConvertMeshToViewer(out_mesh, viewer);
        }

        if (ImGui::Button("write mesh")) {
            meshlib::MeshUtils::WriteMesh(mesh, "data/result/remesh.obj");
        }
    };

    meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
    meshlib::MeshUtils::ConvertMeshToViewer(cmesh, viewer);

    //SplitLongEdges(mesh, 0.6f);

    //meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
    viewer.launch();

    return 0;
}