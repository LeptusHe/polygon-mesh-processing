#pragma once

#include "utils/mesh_utils.h"

namespace meshlib {

class Remesher {
public:
    Remesher();
    void IsotropicRemeshing(Mesh& mesh, float target_edge_len, int num_iteration);

private:
    void SplitLongEdges(Mesh& mesh, float long_edge_len);
    void CollapseShortEdges(Mesh& mesh, float short_edge_len, float long_edge_len);
    bool CollapseHalfEdge(Mesh& mesh, const OpenMesh::HalfedgeHandle& he, float long_edge_sqr_len);
    void EqualizeValences(Mesh& mesh);
    void EqualizeValencesOfEdge(Mesh& mesh, const OpenMesh::EdgeHandle& edge);
    int GetTargetValence(const Mesh& mesh, const OpenMesh::VertexHandle& vh);
    void TangentialRelaxation(Mesh& mesh);
};

} // namespace meshlib