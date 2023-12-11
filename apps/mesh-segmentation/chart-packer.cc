#include "chart-packer.h"

#include <spdlog/spdlog.h>

#include "bounds.h"
#include "xatlas.h"
#include "common.h"

xatlas::UvMeshDecl CreateUVMeshDecal(const Mesh& mesh)
{
    const auto vertex_cnt = mesh.n_vertices();
    const auto uvs = new float[2 * vertex_cnt];

    for (const auto vh : mesh.vertices()) {
        const auto idx = vh.idx();
        auto uv = mesh.texcoord2D(static_cast<Mesh::VertexHandle>(vh));

        uvs[2 * idx + 0] = uv[0];
        uvs[2 * idx + 1] = uv[1];
    }

    const auto indices = new int[3 * mesh.n_faces()];
    for (const auto fh : mesh.faces()) {
        const auto face_idx = fh.idx();

        int offset = 0;
        for (const auto vh : fh.vertices()) {
            indices[3 * face_idx + offset] = vh.idx();
            offset += 1;
        }
    }

    xatlas::UvMeshDecl mesh_decl;
    mesh_decl.faceMaterialData = nullptr;
    mesh_decl.vertexCount = mesh.n_vertices();
    mesh_decl.vertexUvData = uvs;
    mesh_decl.vertexStride = 2 * sizeof(float);
    mesh_decl.indexCount = 3 * mesh.n_faces();
    mesh_decl.indexData = indices;
    mesh_decl.indexFormat = xatlas::IndexFormat::UInt32;
    mesh_decl.indexOffset = 0;

    return mesh_decl;
}

bool SetTextureCoordinate(const xatlas::Atlas *atlas, int mesh_index, Mesh& dst_mesh)
{
    if (atlas == nullptr)
        return false;

    const auto mesh = atlas->meshes[mesh_index];
    Bounds bounds(glm::vec2(mesh.vertexArray[0].uv[0], mesh.vertexArray[0].uv[1]));
    for (int i = 0; i < mesh.vertexCount; ++ i) {
        const auto vert = mesh.vertexArray[i];

        auto uv = glm::vec2(vert.uv[0], vert.uv[1]);
        bounds.Encapsulate(uv);
    }

    const auto uv_bounds_size = bounds.size();
    for (int i = 0; i < mesh.vertexCount; ++ i) {
        const auto vert = mesh.vertexArray[i];

        const auto vert_idx = vert.xref;
        Mesh::VertexHandle vh(static_cast<int>(vert_idx));

        Mesh::TexCoord2D uv(vert.uv[0] / uv_bounds_size.x, vert.uv[1] / uv_bounds_size.y);
        dst_mesh.set_texcoord2D(vh, uv);
    }

    return true;
}

bool MergeChartIntoAtlasMesh(Mesh& atlas_mesh, const Mesh& src_mesh, const xatlas::Mesh& mesh, int chart_idx)
{
    const auto chart = mesh.chartArray[chart_idx];

    for (int i = 0; i < chart.faceCount; ++ i) {
        const auto face_idx = chart.faceArray[i];

        std::vector<uint32_t> vert_indices;

        auto idx0 = mesh.indexArray[3 * face_idx + 0];
        auto idx1 = mesh.indexArray[3 * face_idx + 1];
        auto idx2 = mesh.indexArray[3 * face_idx + 2];
        vert_indices.push_back(idx0);
        vert_indices.push_back(idx1);
        vert_indices.push_back(idx2);

        std::vector<Mesh::VertexHandle> vh_list;
        for (const auto vert_idx : vert_indices) {
            const auto vt = mesh.vertexArray[vert_idx];

            const Mesh::VertexHandle vh(static_cast<int>(vt.xref));
            const auto p = src_mesh.point(vh);

            const auto new_vh = atlas_mesh.add_vertex(p);

            Mesh::TexCoord2D uv(vt.uv[0], vt.uv[1]);
            atlas_mesh.set_texcoord2D(static_cast<Mesh::VertexHandle>(new_vh), uv);

            vh_list.push_back(new_vh);
        }
        atlas_mesh.add_face(vh_list);
    }

    return true;
}

Bounds* CalculateUVBounds(const Mesh& mesh)
{
    Bounds *uv_bounds = nullptr;
    if (mesh.n_vertices() == 0)
        return uv_bounds;

    for (const Mesh::VertexHandle vh : mesh.vertices()) {
        const auto tex_coord = mesh.texcoord2D(vh);

        const auto uv = glm::vec2(tex_coord[0], tex_coord[1]);
        if (uv_bounds == nullptr) {
            uv_bounds = new Bounds(uv);
        }
        uv_bounds->Encapsulate(uv);
    }
    return uv_bounds;
}


std::vector<Mesh> GenerateAtlasMesh(const xatlas::Atlas *atlas, const std::function<const Mesh&(int)>& src_mesh_selector)
{
    if (atlas == nullptr)
        return {};

    spdlog::info("atlas count: {}", atlas->atlasCount);

    std::vector<Mesh> atlas_meshes(atlas->atlasCount);
    for (auto& atlas_mesh : atlas_meshes) {
        atlas_mesh.request_vertex_texcoords2D();
    }

    for (int mesh_index = 0; mesh_index < atlas->meshCount; ++ mesh_index) {
        const auto mesh = atlas->meshes[mesh_index];

        for (int chart_idx = 0; chart_idx < mesh.chartCount; ++ chart_idx) {
            const auto chart = mesh.chartArray[chart_idx];

            auto& atlas_mesh = atlas_meshes[chart.atlasIndex];
            const auto& src_mesh = src_mesh_selector(mesh_index);
            MergeChartIntoAtlasMesh(atlas_mesh, src_mesh, mesh, chart_idx);
        }
    }

    for (auto& atlas_mesh : atlas_meshes) {
        const auto uv_bounds = CalculateUVBounds(atlas_mesh);
        if (!uv_bounds) {
            spdlog::error("failed to calculate uv bounds");
            continue;
        }

        const auto uv_size = uv_bounds->size();
        for (const Mesh::VertexHandle vh : atlas_mesh.vertices()) {
            auto uv = atlas_mesh.texcoord2D(vh);

            uv[0] = uv[0] / uv_size.x;
            uv[1] = uv[1] / uv_size.y;

            atlas_mesh.set_texcoord2D(vh, uv);
        }
        delete uv_bounds;
    }

    return atlas_meshes;
}


bool Pack(const Mesh& mesh, const xatlas::PackOptions& pack_options, std::vector<Mesh>& cluster_meshes)
{
    const auto atlas = xatlas::Create();

    const auto uv_mesh_decl = CreateUVMeshDecal(mesh);

    if (xatlas::AddUvMesh(atlas, uv_mesh_decl) != xatlas::AddMeshError::Success) {
        spdlog::info("failed to add uv mesh");
        return false;
    }
    xatlas::ComputeCharts(atlas);

    // TODO: add options
    std::vector<xatlas::MeshInfo> mesh_infos;
    xatlas::PackCharts(atlas, mesh_infos, pack_options);

    //SetTextureCoordinate(atlas, 0, mesh);
    cluster_meshes = GenerateAtlasMesh(atlas, [&](int mesh_index) -> const Mesh& {
        return mesh;
    });

    // TODO: release memory resources
    // release vertex datas

    return true;
}

bool Pack(const IterativeCluster& cluster, const std::vector<Mesh>& chart_meshes, const xatlas::PackOptions& pack_options, std::vector<Mesh>& cluster_meshes)
{
    const auto atlas = xatlas::Create();

    for (const auto& chart_mesh : chart_meshes) {
        const auto uv_mesh_decl = CreateUVMeshDecal(chart_mesh);
        if (xatlas::AddUvMesh(atlas, uv_mesh_decl) != xatlas::AddMeshError::Success) {
            spdlog::info("failed to add uv mesh");
            return false;
        }
    }

    xatlas::ComputeCharts(atlas);

    const auto chart_center_positions = cluster.GetChartCenterPositions();
    std::vector<xatlas::MeshInfo> mesh_infos;
    for (int i = 0 ; i < chart_meshes.size(); ++ i) {
        const auto chart_center_pos = chart_center_positions[i];
        const auto pos = glm::vec3(chart_center_pos[0], chart_center_pos[1], chart_center_pos[2]);

        xatlas::MeshInfo mesh_info{pos, 0.0f};
        mesh_infos.push_back(mesh_info);
    }

    xatlas::PackCharts(atlas, mesh_infos, pack_options);

    cluster_meshes = GenerateAtlasMesh(atlas, [&](int mesh_index) -> const Mesh& {
        return chart_meshes[mesh_index];
    });

    return true;
}
