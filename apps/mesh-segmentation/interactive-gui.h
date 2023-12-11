#pragma once

int interactive(int argc, char *argv[]);
Mesh MergeMesh(Mesh& merged_mesh, const Mesh& input_mesh);
void WriteChartMesh(const IterativeCluster& cluster, const std::string& file_name, const std::vector<Mesh>& chart_meshes);
