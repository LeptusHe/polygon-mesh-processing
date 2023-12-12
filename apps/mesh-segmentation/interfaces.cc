#include "interfaces.h"
#include "common.h"
#include "iterative-cluster.h"
#include <iostream>

#include "chart-packer.h"
#include "xatlas.h"

Mesh ConstructMesh(const float *vertices, int numVertices, const int *indices, int triangleNum)
{
    assert(vertices != nullptr && indices != nullptr);

    Mesh mesh;
    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();
    mesh.request_face_normals();

    for (int i = 0; i < numVertices; i++)
    {
        mesh.add_vertex(Mesh::Point(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]));
    }

    const int numIndices = 3 * triangleNum;
    for (int i = 0; i < numIndices; i += 3)
    {
        mesh.add_face(mesh.vertex_handle(indices[i]), mesh.vertex_handle(indices[i + 1]), mesh.vertex_handle(indices[i + 2]));
    }

    mesh.update_face_normals();
    return mesh;
}

bool GetSegmentationResult(Mesh& mesh, OpenMesh::FProp<int>& clusterProp, int *clusterIds)
{
    if (clusterIds == nullptr)
        return false;

    for (auto faceHandle : mesh.faces())
    {
        clusterIds[faceHandle.idx()] = clusterProp[faceHandle];
    }
    return true;
}

IterativeCluster::Options InitOptions(const SegmentationOptions& options)
{
    IterativeCluster::Options opt;
    opt.minClusterCnt = std::max(1, options.minClusterCount);
    opt.maxChartArea = std::max(0.0f, options.maxChartArea);
    opt.normalWeight = std::max(0.0f, options.normalWeight);
    opt.maxIterationNum = std::max(1, options.maxIterationNum);
    return opt;
}

bool RunSegmentation(const SegmentationOptions& options, Mesh& mesh, int *clusterIds, int& clusterCnt)
{
    if (clusterIds == nullptr)
        return false;

    auto opt = InitOptions(options);
    OpenMesh::FProp<int> clusterProp(mesh, "cluster");

    try {
        IterativeCluster cluster(mesh, clusterProp);
        cluster.Run(opt);

        clusterCnt = static_cast<int>(cluster.GetClusterCount());
    } catch (std::exception& e) {
        std::cerr << "failed to run segmentation: " << e.what() << std::endl;
        clusterCnt = -1;
        return false;
    }

    return GetSegmentationResult(mesh, clusterProp, clusterIds);
}

int MeshSegmentation(SegmentationOptions options, const float vertices[], int numVertices, const int indices[], int triangleNum, int clusterIds[])
{
    if (vertices == nullptr || indices == nullptr || clusterIds == nullptr)
        return false;

    auto mesh = ConstructMesh(vertices, numVertices, indices, triangleNum);

    int clusterCnt = -1;
    const auto success = RunSegmentation(options, mesh, clusterIds, clusterCnt);
    if (success) {
        return clusterCnt;
    } else {
        return -1;
    }
}


MeshUnwrapper* InitInputMesh(const float vertices[], int numVertices, const int indices[], int triangleNum)
{
    assert(vertices != nullptr && indices != nullptr);

    try {
        const auto mesh = new Mesh();
        mesh->request_vertex_status();
        mesh->request_edge_status();
        mesh->request_face_status();
        mesh->request_face_normals();
        mesh->request_vertex_texcoords2D();

        for (int i = 0; i < numVertices; i++) {
            mesh->add_vertex(Mesh::Point(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]));
        }

        const int numIndices = 3 * triangleNum;
        for (int i = 0; i < numIndices; i += 3) {
            mesh->add_face(mesh->vertex_handle(indices[i]), mesh->vertex_handle(indices[i + 1]), mesh->vertex_handle(indices[i + 2]));
        }
        mesh->update_face_normals();

        const auto mesh_unwrapper = new MeshUnwrapper();
        mesh_unwrapper->input_mesh = mesh;
        return mesh_unwrapper;
    } catch (std::exception& e) {
        std::cerr << "failed to init input mesh because " << e.what() << std::endl;
        return nullptr;
    }
}

ChartPacker::Options InitChartPackOptions(const PackOptions& pack_options)
{
    ChartPacker::Options options;
    options.enable_space_locality = pack_options.enable_space_locality;
    options.xatlas_options.bruteForce = true;
    options.xatlas_options.padding = pack_options.padding;
    options.xatlas_options.texelsPerUnit = pack_options.texel_per_unit;
    options.xatlas_options.resolution = pack_options.resolution;
    options.xatlas_options.bilinear = true;

    options.xatlas_options.createImage = false;

    return options;
}

glm::vec2 CalculateMaxUVSize(int resolution, float texel_per_unit)
{
    auto max_size = std::floor(static_cast<float>(resolution) / texel_per_unit);
    return {max_size, max_size};
}


bool Unwrap(MeshUnwrapper *unwrapper, SegmentationOptions seg_options, PackOptions pack_options)
{
    if (unwrapper == nullptr)
        return false;

    if (unwrapper->input_mesh == nullptr)
        return false;

    auto cluster_opt = InitOptions(seg_options);
    cluster_opt.maxChartArea = 0.0f;
    cluster_opt.enableUVbounds = true;
    cluster_opt.maxUVSize = CalculateMaxUVSize(pack_options.resolution, pack_options.texel_per_unit);

    Mesh& mesh = *unwrapper->input_mesh;
    OpenMesh::FProp<int> clusterProp(mesh, "cluster");

    try {
        const auto cluster = new IterativeCluster(mesh, clusterProp);
        cluster->Run(cluster_opt);

        unwrapper->cluster = cluster;
    } catch (std::exception& e) {
        std::cerr << "failed to run segmentation: " << e.what() << std::endl;
        return false;
    }

    try {
        const auto chart_pack_options = InitChartPackOptions(pack_options);
        const auto chart_packer = new ChartPacker();
        if (!chart_packer->Pack(*unwrapper->cluster, chart_pack_options)) {
            return false;
        }
        unwrapper->chart_packer = chart_packer;
    } catch (std::exception& e) {
        std::cerr << "failed to run chart pack: " << e.what() << std::endl;
        return false;
    }
    return true;
}

int GetAtlasMeshCount(const MeshUnwrapper *unwrapper)
{
    if (unwrapper == nullptr || unwrapper->chart_packer == nullptr)
        return 0;

    const auto& cluster_meshes = unwrapper->chart_packer->GetAtlasMeshes();
    return static_cast<int>(cluster_meshes.size());
}


bool GetAtlasMeshInfo(const MeshUnwrapper *unwrapper, int mesh_index, int& num_vert, int& num_triangle)
{
    if (unwrapper == nullptr || unwrapper->chart_packer == nullptr)
        return false;

    const auto& atlas_meshes = unwrapper->chart_packer->GetAtlasMeshes();
    if (mesh_index >= atlas_meshes.size())
        return false;

    const auto& atlas_mesh = atlas_meshes[mesh_index];
    num_vert = static_cast<int>(atlas_mesh.n_vertices());
    num_triangle = static_cast<int>(atlas_mesh.n_faces());

    return true;
}


bool GetAtlasMeshData(const MeshUnwrapper *unwrapper, int mesh_index, float *positions, float *uvs, int num_vert, int *indices, int num_triangle)
{
    if (unwrapper == nullptr)
        return false;

    const auto& atlas_meshes = unwrapper->chart_packer->GetAtlasMeshes();
    if (mesh_index >= atlas_meshes.size())
        return false;

    if (positions == nullptr || uvs == nullptr || indices == nullptr)
        return false;

    const auto& atlas_mesh = atlas_meshes[mesh_index];
    if (num_vert != atlas_mesh.n_vertices() || num_triangle != atlas_mesh.n_faces()) {
        return false;
    }

    for (int i = 0; i < atlas_mesh.n_vertices(); ++ i) {
        const auto vh = atlas_mesh.vertex_handle(i);
        const auto vert_idx = vh.idx();
        assert(vert_idx < num_vert);

        const OpenMesh::Vec3f pos = atlas_mesh.point(vh);
        positions[3 * vert_idx + 0] = pos[0];
        positions[3 * vert_idx + 1] = pos[1];
        positions[3 * vert_idx + 2] = pos[2];

        const auto uv = atlas_mesh.texcoord2D(vh);
        uvs[2 * vert_idx + 0] = uv[0];
        uvs[2 * vert_idx + 1] = uv[1];
    }

    for (const auto face : atlas_mesh.faces()) {
        const auto faceIndex = face.idx();
        const int baseIndex = 3 * faceIndex;

        int index = 0;
        for (const auto vertex : face.vertices()) {
            const auto vert_idx = vertex.idx();
            assert(baseIndex + index < num_triangle * 3);

            indices[baseIndex + index] = vert_idx;
            index += 1;
        }
        assert(index == 3);
    }

    return true;
}

void DestroyUnwrapper(const MeshUnwrapper *unwapper)
{
    if (unwapper == nullptr)
        return;

    delete unwapper->input_mesh;
    delete unwapper->cluster;
    delete unwapper->chart_packer;
    delete unwapper;
}
