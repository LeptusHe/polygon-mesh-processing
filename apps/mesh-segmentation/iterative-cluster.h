#pragma once
#include "common.h"
#include "bounds.h"

class IterativeCluster {
public:
    struct Options {
        int minClusterCnt = 1;
        int maxIterationNum = 100;
        float maxChartArea = 0.0f;
        float normalWeight = 1.05f;
        bool enableUVbounds = false;
        glm::vec2 maxUVSize;
    };

private:
    const int kInvalidClusterId = -1;

public:
    IterativeCluster(Mesh& mesh, OpenMesh::FProp<int>& clusterProp);
    void Run(const Options& options);
    Mesh::FaceHandle RegionGrowSync(std::vector<Mesh::FaceHandle>& newSeeds);
    float CalculateCost(const Mesh::Normal& chartNormal, Bounds uvBounds, const Mesh::FaceHandle& oldFace, const Mesh::FaceHandle& newFace);
    void InitSeed();
    void ClearClusterProp();
    void UpdateClusterCenters();
    [[nodiscard]] Mesh GetClusterMesh(int clusterId) const;
    [[nodiscard]] bool IsConverged() const;
    [[nodiscard]] std::vector<Mesh::FaceHandle> GetChartCenters() const { return m_seeds; }
    [[nodiscard]] std::vector<Mesh::Point> GetChartCenterPositions() const;
    Mesh::FaceHandle FindCenterOfMesh(const Mesh& mesh);
    void AddSeed(const Mesh::FaceHandle& fh);
    [[nodiscard]] size_t GetClusterCount() const { return m_seeds.size(); }
    [[nodiscard]] const std::vector<Bounds>& GetChartUVBounds() const { return m_chartUVBounds; }

    bool Init(const Options& options);
    bool UpdateCluster();
    [[nodiscard]] std::vector<Mesh> Unwrap() const;

private:
    [[nodiscard]] Bounds CalculateUVBoundsForFace(const Mesh::FaceHandle& faceHandle) const;
    [[nodiscard]] Bounds EncapsulateMeshFace(Bounds& bounds, const Mesh::FaceHandle& face_handle) const;

private:
    Mesh& m_mesh;
    Options m_options;

    OpenMesh::FProp<int>& m_clusterProp;
    std::vector<Mesh> m_chartMeshes;
    std::vector<float> m_chartAreas;
    std::vector<Mesh::Normal> m_clusterNormals;
    std::vector<Bounds> m_chartUVBounds;

    std::vector<OpenMesh::FaceHandle> m_newSeeds;
    std::vector<Mesh::FaceHandle> m_prevSeeds;
    std::vector<Mesh::FaceHandle> m_seeds;

    int m_maxIterCnt = 0;
    int m_restIterCnt = 0;
};