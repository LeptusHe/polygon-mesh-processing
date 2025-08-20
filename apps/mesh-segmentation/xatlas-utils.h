#pragma once

#include "xatlas.h"

namespace xatlas {

auto SortChartMeshByPerimeter(std::vector<ChartInfo>& chart_infos) -> std::vector<ChartInfo>;
std::vector<ChartInfo> CalculatePriority(const MeshInfo& primary_mesh, const std::vector<MeshInfo>& mesh_infos, std::vector<ChartInfo>& chart_infos, float perimeter_weight, float distant_weight);
std::vector<int> RemovePackedChart(const std::vector<int>& chart_ids, const std::vector<int>& packed_chart_ids);

} // namespace xatlas

