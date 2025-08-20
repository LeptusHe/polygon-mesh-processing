#pragma once

#include "xatlas.h"
#include "common.h"
#include "iterative-cluster.h"

class ChartPacker {
public:
    struct Options {
        bool enable_space_locality = true;
        xatlas::PackOptions xatlas_options;
    };

public:
    bool Pack(const IterativeCluster& cluster, const Options& options);
    [[nodiscard]] const std::vector<Mesh>& GetAtlasMeshes() const { return m_atlas_meshes; }

private:
    bool Pack(const Mesh& mesh);
    bool Pack(const IterativeCluster& cluster, const std::vector<Mesh>& chart_meshes);

private:
    Options m_options;
    std::vector<Mesh> m_atlas_meshes;
};

