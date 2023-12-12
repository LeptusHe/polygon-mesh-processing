#pragma once

#include "chart-packer.h"

int interactive(int argc, char *argv[]);

void GeneratePackedClusterMesh(const IterativeCluster& cluster, const ChartPacker::Options& pack_options, const std::string& file_name);
void WriteChartMesh(const IterativeCluster& cluster, const ChartPacker::Options& pack_options, const std::string& file_name, const std::vector<Mesh>& chart_meshes);
Mesh MergeMesh(Mesh& merged_mesh, const Mesh& input_mesh);
