#include "IterativeCluster.h"
#include <queue>

struct Item {
public:
    float cost;
    Mesh::FaceHandle handle;

    Item(float cost, Mesh::FaceHandle handle)
        : cost(cost), handle(handle) {}
};

struct ItemComp {
    bool operator()(const Item& lhs, const Item& rhs) const {
        return lhs.cost > rhs.cost;
    }
};

using Queue = std::priority_queue<Item, std::vector<Item>, ItemComp>;


IterativeCluster::IterativeCluster(Mesh& mesh, OpenMesh::FProp<int>& clusterProp)
    : m_mesh(mesh), m_clusterProp(clusterProp) {}


void IterativeCluster::Run(int k, int maxIter)
{
    m_clusterCount = k;

    // TODO: better seeds
    InitSeed();

    auto restIterCnt = maxIter;
    while (restIterCnt > 0) {
        auto lastFace = RegionGrow();
        if (m_seeds.size() < m_clusterCount) {
            m_seeds.push_back(lastFace);
        }
        restIterCnt -= 1;
    }

}

void IterativeCluster::InitSeed()
{
    m_seeds.clear();
    m_seeds.push_back(m_mesh.face_handle(0));
}

Mesh::FaceHandle IterativeCluster::RegionGrow()
{
    std::vector<Queue> queues(m_seeds.size());
    ClearClusterProp();

    for (int i = 0; i < m_seeds.size(); ++ i) {
        auto seed = m_seeds[i];
        m_clusterProp[seed] = i;
        queues[i].emplace(0.0f, seed);
    }

    Mesh::FaceHandle lastFace;

    while (true) {
        bool notEmpty = false;

        for (int clusterId = 0; clusterId < m_seeds.size(); ++ clusterId) {
            const auto& seed = m_seeds[clusterId];
            auto& queue = queues[clusterId];

            if (queue.empty())
                continue;

            auto item = queue.top();
            queue.pop();

            auto face = item.handle;
            lastFace = face;
            m_clusterProp[face] = clusterId;

            for (auto fh : m_mesh.ff_range(face)) {
                if (m_clusterProp[fh] != kInvalidClusterId)
                    continue;

                auto cost = CalculateCost(face, fh);
                queue.emplace(cost, fh);
            }

            notEmpty = true;
        }

        if (!notEmpty)
            break;
    }
    return lastFace;
}

float IterativeCluster::CalculateCost(const Mesh::FaceHandle& lhs, const Mesh::FaceHandle& rhs)
{
    auto l = m_mesh.calc_face_centroid(lhs);
    auto r = m_mesh.calc_face_centroid(rhs);

    auto d = (l - r).length();

    auto ln = m_mesh.normal(lhs).normalized();
    auto rn = m_mesh.normal(rhs).normalized();
    auto n = ln.dot(rn);

    return (m_lambda - n) * d;
}

void IterativeCluster::ClearClusterProp()
{
    for (auto fh : m_mesh.faces()) {
        m_clusterProp[fh] = kInvalidClusterId;
    }
}

void IterativeCluster::UpdateClusterCenters()
{
    for (int i = 0; i < m_seeds.size(); ++ i) {
    }
}