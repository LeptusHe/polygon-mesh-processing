#include "kmeans.h"
#include <iostream>

void KMeans::Run(int k, int maxIter)
{
    m_clusterCount = k;

    for (int i = 0; i < k; ++ i) {
        m_seeds.push_back(m_mesh.vertex_handle(i));
        m_clusterCenters.push_back(m_mesh.point(m_seeds[i]));
    }

    auto iterCnt = maxIter;
    while (iterCnt > 0) {
        for (const auto vertHandle: m_mesh.vertices()) {
            auto p = m_mesh.point(vertHandle);
            auto clusterId = GetClusterId(p);
            m_clusterProp[vertHandle] = clusterId;
        }

        UpdateClusterCenters();
        if (IsConverged()) {
            break;
        }

        iterCnt -= 1;
    }

    if (iterCnt > 0) {
        std::cout << "KMeans converged in " << maxIter - iterCnt << " iterations." << std::endl;
    } else {
        std::cout << "KMeans did not converge in " << maxIter << " iterations." << std::endl;
    }
}

int KMeans::GetClusterId(const Mesh::Point& p) const
{
    int clusterId = -1;
    float minDist = std::numeric_limits<float>::max();
    for (int i = 0; i < m_clusterCount; ++ i) {
        auto c = m_clusterCenters[i];

        auto d = (p - c).length();
        if (d < minDist) {
            clusterId = static_cast<int>(i);
        }
        minDist = std::min(minDist, d);
    }
    return clusterId;
}

void KMeans::UpdateClusterCenters()
{
    m_prevClusterCenters = m_clusterCenters;

    for (auto& m_clusterCenter : m_clusterCenters) {
        m_clusterCenter = Mesh::Point(0, 0, 0);
    }

    std::vector<int> clusterCounts(m_clusterCount, 0);

    for (const auto vertHandle : m_mesh.vertices()) {
        auto clusterId = m_clusterProp[vertHandle];

        clusterCounts[clusterId] += 1;
        m_clusterCenters[clusterId] += m_mesh.point(vertHandle);
    }

    for (int i = 0; i < m_clusterCount; ++ i) {
        m_clusterCenters[i] /= clusterCounts[i];
    }
}

bool KMeans::IsConverged() const
{
    bool converged = true;
    for (size_t i = 0; i < m_clusterCenters.size(); ++ i) {
        auto prev = m_prevClusterCenters[i];
        auto cur = m_clusterCenters[i];
        if ((prev - cur).length() > 1e-5) {
            converged = false;
            break;
        }
    }
    return converged;
}