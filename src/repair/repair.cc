#include "repair.h"
#include "utils/mesh_utils.h"
#include "utils/vertex_merger.h"
#include <spdlog/spdlog.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

bool IsSmallHole(CMesh::halfedge_index h, const CMesh& mesh, double max_hole_diam, int max_num_hole_edges)
{
    int num_hole_edges = 0;

    CGAL::Bbox_3 hole_bbox;
    for (auto hc : CGAL::halfedges_around_face(h, mesh)) {
        const CMesh::Point& p = mesh.point(target(hc, mesh));
        hole_bbox += p.bbox();

        ++num_hole_edges;

        // Exit early, to avoid unnecessary traversal of large holes
        if (num_hole_edges > max_num_hole_edges)
            return false;

        if (hole_bbox.xmax() - hole_bbox.xmin() > max_hole_diam)
            return false;
        if (hole_bbox.ymax() - hole_bbox.ymin() > max_hole_diam)
            return false;
        if (hole_bbox.zmax() - hole_bbox.zmin() > max_hole_diam)
            return false;
  }
  return true;
}

int FillSmallHoles(CMesh& mesh, double max_hole_diam, int max_num_hole_edge)
{
    int hole_cnt = 0;
    if (max_num_hole_edge <= 0 || max_hole_diam <= 0)
        return hole_cnt;

    std::vector<CMesh::halfedge_index> border_cycles;
    pmp::extract_boundary_cycles(mesh, std::back_inserter(border_cycles));

    for (auto he : border_cycles) {
        if (!IsSmallHole(he, mesh, max_hole_diam, max_num_hole_edge))
            continue;

        std::vector<CMesh::face_index> patch_facets;
        std::vector<CMesh::vertex_index> patch_vertices;

        auto tuple = pmp::triangulate_refine_and_fair_hole(mesh, he);
        if (std::get<0>(tuple)) {
            hole_cnt += 1;
        }
    }
    return hole_cnt;
}

CMesh Repair(const CMesh& mesh)
{
    std::vector<CMesh::Point> points;
    std::vector<std::vector<std::size_t>> polygon;
    pmp::polygon_mesh_to_polygon_soup(mesh, points, polygon);

    int num_removed = pmp::merge_duplicate_points_in_polygon_soup(points, polygon);
    num_removed = RemoveDuplicationVertex(points, polygon);
    auto reverse_faces = FixInvalidOrientation(points, polygon);

    auto another_num_removed = pmp::merge_duplicate_points_in_polygon_soup(points, polygon);
    auto num_faces = pmp::merge_duplicate_polygons_in_polygon_soup(points, polygon);
    pmp::repair_polygon_soup(points, polygon);
    if (!pmp::orient_polygon_soup(points, polygon)) {
        spdlog::info("invalid orient");
    }

    CMesh clean_mesh;
    pmp::polygon_soup_to_polygon_mesh(points, polygon, clean_mesh);
    return clean_mesh;
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


bool CollapseBoundaryEdge(Mesh& mesh)
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
    return found;
}

bool CollapseBoundaryEdge(CMesh& cmesh)
{
    auto mesh = meshlib::ConvertToMesh(cmesh);

    int count = 0;
    bool found = true;
    do {
        found = CollapseBoundaryEdge(mesh);
        count += 1;
    } while (found);

    cmesh = meshlib::ConvertOpenMeshToSurfaceMesh(mesh);
    return found;
}
