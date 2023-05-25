#pragma once

#include <string>
#include "MeshUtils.h"

namespace meshlib {

bool LoadMesh(const std::string &filePath, Mesh &mesh, OpenMesh::IO::Options &opt);

} // namespace meshlib
