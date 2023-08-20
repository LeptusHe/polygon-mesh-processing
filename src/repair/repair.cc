#include "repair.h"
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>

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