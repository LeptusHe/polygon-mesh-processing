#pragma once

#include "utils/mesh_utils.h"

class VertexBaker {
public:
    explicit VertexBaker(Mesh& mesh);

    void Solve();

private:
    void CalculateCoefficientMatrix();

private:
    Mesh& mesh_;
};
