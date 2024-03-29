#include "IterativeCluster.h"
#include <queue>
#include <iostream>

struct Item {
public:
    float cost;
    Mesh::FaceHandle handle;
    int clusterId = -1;
    bool seed = false;

    Item(float cost, Mesh::FaceHandle handle)
        : cost(cost), handle(handle) {}

    Item(float cost, Mesh::FaceHandle handle, int clusterId)
        : cost(cost), handle(handle), clusterId(clusterId) {}

    Item(float cost, Mesh::FaceHandle handle, int clusterId, bool seed)
            : cost(cost), handle(handle), clusterId(clusterId), seed(seed) {}
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
    Init(k, lambda, maxIter);

    while (true) {
        if (!UpdateCluster())
            break;
    }

    if (m_restIterCnt == 0) {
        std::cout << "IterativeCluster: reach max iteration count: " << maxIter << std::endl;
    } else {
        std::cout << "IterativeCluster: converge in " << maxIter - m_restIterCnt << " iteration"  << std::endl;
    }
}

bool IterativeCluster::Init(int k, float lambda, int maxIter)
{
    m_clusterCount = std::min(static_cast<int>(m_mesh.n_faces()), k);
    m_lambda = std::max(1.0f, lambda);

    m_maxIterCnt = maxIter;
    m_restIterCnt = maxIter;

    InitSeed();
    return true;
}


bool IterativeCluster::UpdateCluster()
{
    if (m_newSeeds.empty() && m_restIterCnt <= 0)
        return false;

    m_prevSeeds = m_seeds;

    std::cout << "iteration: " << m_maxIterCnt - m_restIterCnt << "\n\n" << std::endl;

    m_newSeeds.clear();
    //auto lastFace = RegionGrow();
    auto lastFace = RegionGrowSync(m_newSeeds);
    UpdateClusterCenters();
    //UpdateClusterCentersByPos();

    if (m_newSeeds.empty()) {
        if (m_seeds.size() < m_clusterCount) {
            AddSeed(lastFace);
        }
    } else {
        for (auto newSeed: m_newSeeds) {
            AddSeed(newSeed);
        }
    }

    if (m_newSeeds.empty() && IsConverged())
        return true;

    m_restIterCnt -= 1;

    std::cout << "iterative count: " << m_maxIterCnt - m_restIterCnt << std::endl;
    std::cout << "IterativeCluster: " << m_seeds.size() << " clusters" << std::endl;
    return true;
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

    std::vector<OpenMesh::FProp<bool>> visitedProps;
    for (int i = 0; i < m_seeds.size(); ++ i) {
        visitedProps.emplace_back(m_mesh);
    }

    for (int i = 0; i < m_seeds.size(); ++ i) {
        auto seed = m_seeds[i];
        m_clusterProp[seed] = i;
        queues[i].emplace(0.0f, seed);

        visitedProps[i][seed] = true;
    }
    m_clusterNormals = std::vector<Mesh::Normal>(m_seeds.size(), Mesh::Normal(0, 0, 0));

    Mesh::FaceHandle lastFace;

    while (true) {
        bool notEmpty = false;

        for (int clusterId = 0; clusterId < m_seeds.size(); ++ clusterId) {
            auto& queue = queues[clusterId];

            if (queue.empty())
                continue;

            Item item = queue.top();
            while (true) {
                item = queue.top();
                queue.pop();

                if (m_clusterProp[item.handle] == clusterId || m_clusterProp[item.handle] == kInvalidClusterId)
                    break;

                if (queue.empty())
                    break;
            }

            auto face = item.handle;
            lastFace = face;
            m_clusterProp[face] = clusterId;

            if (clusterId == 0) {
                //std::cout << "f: " << face.idx() << " cost: " << item.cost << std::endl;
            }

            // Note：更新chart的法线
            m_clusterNormals[clusterId] += m_mesh.normal(face);
            m_clusterNormals[clusterId].normalize();

            for (auto fh : m_mesh.ff_range(face)) {
                if (m_clusterProp[fh] != kInvalidClusterId)
                    continue;

                if (visitedProps[clusterId][fh])
                    continue;

                auto cost = CalculateCost(m_clusterNormals[clusterId], face, fh);
                queue.emplace(cost, fh);
                visitedProps[clusterId][fh] = true;
            }

            notEmpty = true;
        }

        if (!notEmpty)
            break;
    }
    return lastFace;
}

Mesh::FaceHandle IterativeCluster::RegionGrowSync(std::vector<Mesh::FaceHandle>& newSeeds)
{
    PriorityQueue queue;
    ClearClusterProp();

    const size_t seedCount = m_seeds.size();

    std::vector<OpenMesh::FProp<bool>> visitedProps;
    for (int i = 0; i < seedCount; ++ i) {
        visitedProps.emplace_back(m_mesh);
    }

    m_chartMeshes.clear();
    for (int i = 0; i < seedCount; ++ i) {
        auto chartMesh = m_mesh;
        for (const auto face : chartMesh.faces()) {
            chartMesh.delete_face(face, false);
        }
        chartMesh.garbage_collection(false, true, true);

        //std::cout << "vertex n: " << chartMesh.n_vertices() << std::endl;
        //std::cout << "face n: " << chartMesh.n_faces() << std::endl;
        //std::cout << "half edge n: " << chartMesh.n_halfedges() << std::endl;

        m_chartMeshes.push_back(chartMesh);
    }

    for (int clusterId = 0; clusterId < seedCount; ++ clusterId) {
        auto seed = m_seeds[clusterId];
        m_clusterProp[seed] = clusterId;
        queue.emplace(0.0f, seed, clusterId, true);
        visitedProps[clusterId][seed] = true;
    }
    m_clusterNormals = std::vector<Mesh::Normal>(seedCount, Mesh::Normal(0, 0, 0));

    int count = 0;

    Mesh::FaceHandle lastFace;
    while (!queue.empty()) {
        auto item = queue.top();
        queue.pop();

        auto fh = item.handle;
        if (!item.seed && m_clusterProp[fh] != kInvalidClusterId)
            continue;

        count += 1;
        if (count > m_mesh.n_faces()) {
            std::cout << "invalid queue: " << count  << std::endl;
        }

        const int clusterId = item.clusterId;

        // add face to chart
        std::vector<Mesh::VertexHandle> fhs;
        for (const auto vh : m_mesh.fv_range(fh)) {
            if (!m_chartMeshes[clusterId].is_boundary(vh)) {
                std::cout <<  "invalid boundary vertex" << std::endl;
            }
            fhs.push_back(vh);
        }

        auto& chartMesh = m_chartMeshes[clusterId];
        auto newFH = chartMesh.add_face(fhs);
        if (!newFH.is_valid()) {
            std::cout << "invalid add face: " << newFH.idx() << std::endl;
            std::cout << "half edges: " << chartMesh.n_halfedges() << std::endl;
            std::cout << "edges: " << chartMesh.n_edges() << std::endl;
            std::cout << "faces: " << chartMesh.n_faces() << std::endl;
        }

        bool isClosed = true;
        for (const auto he : chartMesh.halfedges()) {
            if (chartMesh.is_boundary(he)) {
                isClosed = false;
                break;
            }
        }
        if (isClosed) {
            chartMesh.delete_face(newFH);
            newSeeds.push_back(fh);
            std::cout << "close mesh: " << fh.idx() << std::endl;
            continue;
        }

        // update data

        m_clusterProp[fh] = clusterId;
        m_clusterNormals[clusterId] += m_mesh.normal(fh);
        m_clusterNormals[clusterId].normalize();
        lastFace = fh;

        for (auto f : m_mesh.ff_range(fh)) {
            if (m_clusterProp[f] != kInvalidClusterId)
                continue;

            if (visitedProps[clusterId][f])
                continue;

            auto cost = CalculateCost(m_clusterNormals[clusterId], fh, f);
            queue.emplace(item.cost + cost, f, clusterId);
            visitedProps[clusterId][f] = true;
        }
    }

    if (!lastFace.is_valid()) {
        std::cout << lastFace.idx() << std::endl;
    }

    if (newSeeds.empty()) {
        for (auto fh: m_mesh.faces()) {
            if (m_clusterProp[fh] == kInvalidClusterId) {
                std::cout << "add not connected sed: " << fh.idx() << std::endl;
                newSeeds.push_back(fh);
                break;
            }
        }
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
        m_seeds[i] = FindCenterOfMesh(mesh);
    }
}

Mesh::FaceHandle IterativeCluster::FindCenterOfMesh(const Mesh& mesh)
{
    auto visited = OpenMesh::FProp<bool>(mesh);

    PriorityQueue queue;
    for (auto fh : mesh.faces()) {
        if (mesh.is_boundary(fh)) {
            queue.emplace(0.0f, fh, 0, true);
        }
    }

    Mesh::FaceHandle lastFace;

    int cnt = 0;

    while (!queue.empty()) {
        auto item = queue.top();
        queue.pop();

        if (!item.seed && visited[item.handle])
            continue;

        cnt ++;
        if (cnt > mesh.n_faces()) {
            std::cout << "queue: " << cnt << std::endl;
        }

        auto face = item.handle;
        visited[face] = true;
        lastFace = face;

        for (auto fh : mesh.ff_range(face)) {
            if (visited[fh])
                continue;

            if (!fh.is_valid()) {
                std::cout << "invalid in enqueue: " << lastFace.idx() << std::endl;
            }

            auto l = mesh.calc_face_centroid(face);
            auto r = mesh.calc_face_centroid(fh);
            auto cost = (l - r).length();

            queue.emplace(item.cost + cost, fh);
        }
    }

    if (!lastFace.is_valid()) {
        std::cout << "invalid for last face: " << lastFace.idx() << std::endl;
    }
    return lastFace;
}

void IterativeCluster::UpdateClusterCentersByPos()
{
    for (int i = 0; i < m_seeds.size(); ++ i) {
        auto mesh = GetClusterMesh(i);

        OpenMesh::Vec3f center(0, 0, 0);
        for (const auto fh : mesh.faces()) {
            center += mesh.calc_face_centroid(fh);
        }
        center /= mesh.n_faces();

        Mesh::FaceHandle seed;
        float minDist = std::numeric_limits<float>::max();

        for (const auto fh : mesh.faces()) {
            auto fpos = mesh.calc_face_centroid(fh);
            float dist = (fpos - center).length();

            if (dist < minDist) {
                minDist = dist;
                seed = fh;
            }
        }
        m_seeds[i] = seed;
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

void IterativeCluster::AddSeed(const Mesh::FaceHandle& fh)
{
    for (const auto& seed : m_seeds) {
        if (seed == fh) {
            std::cout << "ignore exist seed: " << fh.idx() << std::endl;
            return;
        }
    }
    m_seeds.push_back(fh);
}