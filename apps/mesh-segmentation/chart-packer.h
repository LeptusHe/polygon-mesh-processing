#pragma once

#include "xatlas.h"
#include "common.h"

class ChartPacker {

};

bool Pack(Mesh& mesh, const xatlas::PackOptions& pack_options);
bool Pack(const Mesh& mesh, const xatlas::PackOptions& pack_options, std::vector<Mesh>& cluster_meshes);
