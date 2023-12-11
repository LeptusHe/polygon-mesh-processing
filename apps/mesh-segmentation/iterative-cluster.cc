#include "iterative-cluster.h"
#include <queue>
#include <iostream>

#include <spdlog/spdlog.h>

#define ENABLE_LOG 1

struct Item {
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
    : m_mesh(mesh), m_clusterProp(clusterProp)
{
}


void IterativeCluster::Run(const Options& options)
{
    Init(options);

    while (true) {
        if (!UpdateCluster())
            break;
    }

#if ENABLE_LOG
    if (m_restIterCnt == 0) {
        spdlog::info("IterativeCluster: reach max iteration count: {}", m_options.maxIterationNum);
    } else {
        spdlog::info("IterativeCluster: converge in {} iteration", m_options.maxIterationNum - m_restIterCnt);
    }
#endif
}

bool IterativeCluster::Init(const Options& options)
{
    m_options = options;
    m_options.minClusterCnt = std::min(static_cast<int>(m_mesh.n_faces()), m_options.minClusterCnt);
    m_options.maxIterationNum = std::max(0, m_options.maxIterationNum);
    m_options.normalWeight = std::max(1.0f, m_options.normalWeight);
    m_options.enableUVbounds = options.enableUVbounds;
    m_options.maxUVSize = options.maxUVSize;

    m_maxIterCnt = m_options.maxIterationNum;
    m_restIterCnt = m_options.maxIterationNum;

    InitSeed();

    return true;
}


bool IterativeCluster::UpdateCluster()
{
    if (m_newSeeds.empty() && m_restIterCnt <= 0)
        return false;

    m_prevSeeds = m_seeds;

    m_newSeeds.clear();
    //auto lastFace = RegionGrow();
    const auto lastFace = RegionGrowSync(m_newSeeds);
    UpdateClusterCenters();
    //UpdateClusterCentersByPos();

    if (m_newSeeds.empty()) {
        if (m_seeds.size() < m_options.minClusterCnt) {
            AddSeed(lastFace);
        }
    } else {
        for (auto newSeed: m_newSeeds) {
            AddSeed(newSeed);
        }
    }

    if (m_newSeeds.empty() && IsConverged())
        return false;

    m_restIterCnt -= 1;

#if ENABLE_LOG
    std::cout << "iteration: " << m_maxIterCnt - m_restIterCnt << "\n\n" << std::endl;
    std::cout << "iterative count: " << m_maxIterCnt - m_restIterCnt << std::endl;
    std::cout << "IterativeCluster: " << m_seeds.size() << " clusters" << std::endl;
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
#endif

    return true;
}

void IterativeCluster::InitSeed()
{
    m_seeds.clear();
    m_seeds.push_back(m_mesh.face_handle(0));
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
    m_chartAreas.clear();
    m_chartUVBounds.clear();
    for (int i = 0; i < seedCount; ++ i) {
        auto chartMesh = m_mesh;
        for (const auto face : chartMesh.faces()) {
            chartMesh.delete_face(face, false);
        }
        chartMesh.garbage_collection(false, true, true);

        m_chartMeshes.push_back(chartMesh);
        m_chartAreas.push_back(0.0f);
    }

    for (int clusterId = 0; clusterId < seedCount; ++ clusterId) {
        auto seed = m_seeds[clusterId];
        m_clusterProp[seed] = clusterId;

        auto uvBounds = CalculateUVBoundsForFace(seed);
        m_chartUVBounds.emplace_back(uvBounds);

        queue.emplace(0.0f, seed, clusterId, true);
        visitedProps[clusterId][seed] = true;
    }
    m_clusterNormals = std::vector(seedCount, Mesh::Normal(0, 0, 0));

    int count = 0;
    Mesh::FaceHandle lastFace;
    while (!queue.empty()) {
        const auto item = queue.top();
        queue.pop();

        auto fh = item.handle;
        if (!item.seed && m_clusterProp[fh] != kInvalidClusterId)
            continue;

        if (item.cost == std::numeric_limits<float>::max()) {
            spdlog::warn("skip invalid item");
            continue;
        }

        count += 1;
        if (count > m_mesh.n_faces()) {
            //std::cout << "invalid queue: " << count  << std::endl;
        }

        const int clusterId = item.clusterId;
        if (m_options.maxChartArea > 0.0f && m_chartAreas[clusterId] > m_options.maxChartArea) {
            continue;
        }

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
        auto uvBounds = m_chartUVBounds[clusterId];
        uvBounds = EncapsulateMeshFace(uvBounds, fh);
        m_chartUVBounds[clusterId] = uvBounds;

        m_clusterProp[fh] = clusterId;
        m_clusterNormals[clusterId] += m_mesh.normal(fh);
        m_clusterNormals[clusterId].normalize();
        m_chartAreas[clusterId] += m_mesh.calc_face_area(fh);

        lastFace = fh;
        for (auto f : m_mesh.ff_range(fh)) {
            if (m_clusterProp[f] != kInvalidClusterId)
                continue;

            if (visitedProps[clusterId][f])
                continue;

            auto cost = CalculateCost(m_clusterNormals[clusterId], uvBounds, fh, f);
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


float IterativeCluster::CalculateCost(const Mesh::Normal& chartNormal, Bounds uvBounds, const Mesh::FaceHandle& oldFace, const Mesh::FaceHandle& newFace)
{
    const auto l = m_mesh.calc_face_centroid(oldFace);
    const auto r = m_mesh.calc_face_centroid(newFace);
    const auto d = (l - r).length();

    const auto rn = m_mesh.normal(newFace).normalized();
    const auto n = chartNormal.dot(rn);

    float uv_bounds_cost = 0;
    if (m_options.enableUVbounds) {
        const auto old_bounds_area = uvBounds.area();
        for (const auto v : m_mesh.fv_range(newFace)) {
            auto p = m_mesh.point(v);
            uvBounds.Encapsulate(glm::vec2(p[0], p[2]));
        }
        const auto new_bounds_area = uvBounds.area();
        const auto max_area = m_options.maxUVSize.x * m_options.maxUVSize.y;

        const auto size = uvBounds.size();
        if (size.x > m_options.maxUVSize.x || size.y > m_options.maxUVSize.y) {
#if ENABLE_LOG
            //std::cout << "selected triangle exceeds the maximum uv size" << std::endl;
#endif

            return std::numeric_limits<float>::max();
        } else {
            uv_bounds_cost = (new_bounds_area - old_bounds_area) / (max_area);
        }
    }

    return (m_options.normalWeight - n) * d + uv_bounds_cost * 1.0f;
}

void IterativeCluster::ClearClusterProp()
{
    for (auto fh : m_mesh.faces()) {
        m_clusterProp[fh] = kInvalidClusterId;
    }
}

Mesh IterativeCluster::GetClusterMesh(int clusterId)
{
    Mesh mesh = m_mesh;

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

std::vector<Mesh::Point> IterativeCluster::GetChartCenterPositions() const
{
    std::vector<Mesh::Point> positions;
    for (const auto fh : m_seeds) {
        const auto pos = m_mesh.calc_face_centroid(fh);
        positions.push_back(pos);
    }
    return positions;
}


bool IterativeCluster::IsConverged() const
{
    if (m_seeds.size() < m_options.minClusterCnt)
        return false;

    if (m_prevSeeds.size() != m_seeds.size())
        return false;

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

Bounds IterativeCluster::CalculateUVBoundsForFace(const Mesh::FaceHandle& faceHandle) const
{
    auto vhIter = m_mesh.fv_iter(faceHandle);
    auto p = m_mesh.point(*vhIter);

    Bounds bounds(glm::vec2(p[0], p[2]));
    for (const auto& vh : m_mesh.fv_range(faceHandle)) {
        p = m_mesh.point(vh);
        bounds.Encapsulate(glm::vec2(p[0], p[2]));
    }
    return bounds;
}

Bounds IterativeCluster::EncapsulateMeshFace(Bounds& bounds, const Mesh::FaceHandle& face_handle) const
{
    for (const auto vh : m_mesh.fv_range(face_handle)) {
        auto p = m_mesh.point(static_cast<Mesh::VertexHandle>(vh));
        bounds.Encapsulate(glm::vec2(p[0], p[2]));
    }
    return bounds;
}

std::vector<Mesh> IterativeCluster::Unwrap()
{
    if (!m_options.enableUVbounds) {
        spdlog::error("failed to unwrap uv because uv bounds not enabled");
        return {};
    }

    std::vector<Mesh> chart_meshes;
    const auto chart_cnt = GetClusterCount();
    for (int chart_idx = 0; chart_idx < chart_cnt; ++ chart_idx) {
        auto chart_mesh = GetClusterMesh(chart_idx);
        chart_mesh.garbage_collection();
        chart_mesh.request_vertex_texcoords2D();

        const auto& chart_uv_bounds = m_chartUVBounds[chart_idx];
        const auto& uv_size = chart_uv_bounds.size();
        for (const auto vh : chart_mesh.vertices()) {
            const auto p = chart_mesh.point(vh);

            glm::vec2 uv(p[0], p[2]);
            uv = uv - chart_uv_bounds.min;
            uv.x = uv.x / uv_size.x;
            uv.y = uv.y / uv_size.y;

            Mesh::TexCoord2D coord(2 * chart_idx + uv.x, uv.y);
            chart_mesh.set_texcoord2D(vh, coord);
        }
        chart_meshes.emplace_back(chart_mesh);
    }
    return chart_meshes;
}
