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

using PriorityQueue = std::priority_queue<Item, std::vector<Item>, ItemComp>;


IterativeCluster::IterativeCluster(Mesh& mesh, OpenMesh::FProp<int>& clusterProp)
    : m_mesh(mesh), m_clusterProp(clusterProp) {}


void IterativeCluster::Run(int k, float lambda, int maxIter)
{
    m_clusterCount = k;
    m_lambda = std::clamp(lambda, 0.0f, 1.0f);

    InitSeed();

    auto restIterCnt = maxIter;
    while (restIterCnt > 0) {
        m_prevSeeds = m_seeds;

        auto lastFace = RegionGrow();
        UpdateClusterCenters();
        if (IsConverged())
            break;

        if (m_seeds.size() < m_clusterCount) {
            m_seeds.push_back(lastFace);
        }
        restIterCnt -= 1;

        std::cout << "iterative count: " << maxIter - restIterCnt << std::endl;
        std::cout << "IterativeCluster: " << m_seeds.size() << " clusters" << std::endl;
    }

    if (restIterCnt == 0) {
        std::cout << "IterativeCluster: reach max iteration count: " << maxIter << std::endl;
    } else {
        std::cout << "IterativeCluster: converge in " << maxIter - restIterCnt << " iteration"  << std::endl;
    }
}

void IterativeCluster::InitSeed()
{
    m_seeds.clear();
    m_seeds.push_back(m_mesh.face_handle(0));
}

Mesh::FaceHandle IterativeCluster::RegionGrow()
{
    std::vector<PriorityQueue> queues(m_seeds.size());
    ClearClusterProp();

    for (int i = 0; i < m_seeds.size(); ++ i) {
        auto seed = m_seeds[i];
        m_clusterProp[seed] = i;
        queues[i].emplace(0.0f, seed);
    }
    m_clusterNormals = std::vector<Mesh::Normal>(m_seeds.size(), Mesh::Normal(0, 0, 0));

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

            // Note：更新chart的法线
            m_clusterNormals[clusterId] += m_mesh.normal(face);
            m_clusterNormals[clusterId].normalize();

            for (auto fh : m_mesh.ff_range(face)) {
                if (m_clusterProp[fh] != kInvalidClusterId)
                    continue;

                auto cost = CalculateCost(m_clusterNormals[clusterId], face, fh);
                queue.emplace(cost, fh);
            }

            notEmpty = true;
        }

        if (!notEmpty)
            break;
    }
    return lastFace;
}


float IterativeCluster::CalculateCost(const Mesh::Normal& chartNormal, const Mesh::FaceHandle& oldFace, const Mesh::FaceHandle& newFace)
{
    auto l = m_mesh.calc_face_centroid(oldFace);
    auto r = m_mesh.calc_face_centroid(newFace);
    auto d = (l - r).length();

    auto rn = m_mesh.normal(newFace).normalized();
    auto n = chartNormal.dot(rn);

    return (m_lambda - n) * d;
}

void IterativeCluster::ClearClusterProp()
{
    for (auto fh : m_mesh.faces()) {
        m_clusterProp[fh] = kInvalidClusterId;
    }
}

Mesh IterativeCluster::GetClusterMesh(int clusterId)
{
    auto mesh = m_mesh;

    for (auto fh : mesh.faces()) {
        if (m_clusterProp[fh] != clusterId) {
            mesh.delete_face(fh);
        }
    }
    return mesh;
}

void IterativeCluster::UpdateClusterCenters()
{
    for (int i = 0; i < m_seeds.size(); ++ i) {
        auto mesh = GetClusterMesh(i);
        auto visited = OpenMesh::FProp<bool>(mesh);

        PriorityQueue queue;
        for (auto fh : mesh.faces()) {
            if (mesh.is_boundary(fh)) {
                queue.emplace(0.0f, fh);
            }
        }

        Mesh::FaceHandle lastFace;
        while (!queue.empty()) {
            auto item = queue.top();

            queue.pop();

            auto face = item.handle;
            visited[face] = true;
            lastFace = face;

            for (auto fh : mesh.ff_range(face)) {
                if (visited[fh])
                    continue;

                auto l = mesh.calc_face_centroid(face);
                auto r = mesh.calc_face_centroid(fh);
                auto cost = (l - r).length();

                queue.emplace(cost, fh);
            }
        }

        m_seeds[i] = lastFace;
    }
}

bool IterativeCluster::IsConverged() const
{
    if (m_seeds.size() < m_clusterCount)
        return false;

    if (m_prevSeeds.size() != m_seeds.size())
        return false;

    std::cout << "prev seeds: ";
    for (int i = 0; i < m_prevSeeds.size(); ++i) {
        std::cout << m_prevSeeds[i].idx() << " ";
    }
    std::cout << std::endl;

    std::cout << "cur  seeds: ";
    for (int i = 0; i < m_seeds.size(); ++ i) {
        std::cout << m_seeds[i].idx() << " ";
    }
    std::cout << std::endl;

    for (int i = 0; i < m_seeds.size(); ++ i) {
        auto prevCenter = m_prevSeeds[i];
        auto curCenter = m_seeds[i];
        if (prevCenter != curCenter)
            return false;
    }
    return true;
}