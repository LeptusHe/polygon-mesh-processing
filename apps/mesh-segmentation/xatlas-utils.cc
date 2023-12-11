#include "xatlas-utils.h"

#include <unordered_set>

namespace xatlas {

std::vector<ChartInfo> CalculatePriority(const MeshInfo& primary_mesh, const std::vector<MeshInfo>& mesh_infos, std::vector<ChartInfo>& chart_infos, float perimeter_weight, float distant_weight)
{
    float max_dist = 0.0f;
    for (ChartInfo& chart_info : chart_infos) {
        const auto chart_id = chart_info.chart_id;

        const auto& center = mesh_infos[chart_id].center_pos;
        auto distant = glm::distance(center, primary_mesh.center_pos);
        chart_info.distant = distant;
        max_dist = std::max(distant, max_dist);
    }

    for (auto& chart_info : chart_infos) {
        const float perimeter_cost = perimeter_weight * chart_info.chart_perimeter;
        const float dist_cost = distant_weight * chart_info.distant;
        const float cost = perimeter_cost + dist_cost;
        chart_info.cost = cost;
    }

    std::sort(std::begin(chart_infos), std::end(chart_infos), [](const ChartInfo& lhs, const ChartInfo& rhs) {
        return lhs.cost - rhs.cost;
    });
    return chart_infos;
}

auto SortChartMesh(const std::vector<float>& scores) -> std::vector<std::pair<int, float>>
{
    std::vector<std::pair<int, float>> mesh_score_pairs;
    for (int i = 0; i < scores.size(); ++ i) {
        auto pair = std::make_pair(i, scores[i]);
        mesh_score_pairs.push_back(pair);
    }

    std::sort(std::begin(mesh_score_pairs), std::end(mesh_score_pairs), [](const std::pair<int, float>& lhs, const std::pair<int, float>& rhs) {
        return lhs.second - rhs.second;
    });

    return mesh_score_pairs;
}

auto SortChartMeshByPerimeter(std::vector<ChartInfo>& chart_infos) -> std::vector<ChartInfo>
{
    std::sort(std::begin(chart_infos), std::end(chart_infos), [](const ChartInfo& lhs, const ChartInfo& rhs) {
        return lhs.chart_perimeter - rhs.chart_perimeter;
    });
    return chart_infos;
}

std::vector<int> RemovePackedChart(const std::vector<int>& chart_ids, const std::vector<int>& packed_chart_ids)
{
    std::unordered_set<int> packed_chart_id_set;
    for (const auto chart_id : packed_chart_ids) {
        packed_chart_id_set.insert(chart_id);
    }

    std::vector<int> result;
    for (const auto& chart_id : chart_ids) {
        if (packed_chart_id_set.find(chart_id) == std::end(packed_chart_id_set)) {
            result.push_back(chart_id);
        }
    }
    return result;
}

} // namespace xatlas