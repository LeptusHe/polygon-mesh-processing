#include "vertex-baker.h"

VertexBaker::VertexBaker(Mesh& mesh)
    : mesh_(mesh)
{

}


void VertexBaker::CalculateCoefficientMatrix()
{
    for (const auto fh : mesh_.faces()) {
        for (const auto vh : mesh_.fv_range(fh)) {

        }
    }
}