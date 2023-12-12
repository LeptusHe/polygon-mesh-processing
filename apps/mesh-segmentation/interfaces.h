#pragma once

#include "chart-packer.h"
#include "iterative-cluster.h"

#ifndef MESHLIB_EXPORT_API
    #define MESHLIB_EXPORT_API 0
#endif

#ifndef MESHLIB_IMPORT_API
    #define MESHLIB_IMPORT_API 0
#endif

#ifndef MESHLIB_API
    #if MESHLIB_EXPORT_API
        #ifdef _MSC_VER
            #define MESHLIB_API __declspec(dllexport)
        #else
            #define MESHLIB_API __attribute__((visibility("default")))
        #endif
    #elif MESHLIB_IMPORT_API
        #ifdef _MSC_VER
            #define MESHLIB_API __declspec(dllimport)
        #else
            #define MESHLIB_API
        #endif
    #else
        #define MESHLIB_API
    #endif
#endif

//extern "C" {

struct MeshUnwrapper {
    Mesh *input_mesh = nullptr;
    const IterativeCluster *cluster = nullptr;
    const ChartPacker *chart_packer = nullptr;
};

struct SegmentationOptions {
    int minClusterCount = 1;
    float maxChartArea = 1000.0f;
    float normalWeight = 1.0f;
    int maxIterationNum = 100;
};

struct PackOptions {
    bool enable_space_locality = true;
    int resolution = 1024;
    int padding = 2;
    float texel_per_unit = 64;
};


MESHLIB_API int MeshSegmentation(SegmentationOptions options, const float vertices[], int numVertices, const int indices[], int triangleNum, int clusterIds[]);

MESHLIB_API MeshUnwrapper* InitInputMesh(const float vertices[], int numVertices, const int indices[], int triangleNum);
MESHLIB_API bool Unwrap(MeshUnwrapper *unwrapper, SegmentationOptions seg_options, PackOptions pack_options);
MESHLIB_API int GetAtlasMeshCount(const MeshUnwrapper *unwrapper);
MESHLIB_API bool GetAtlasMeshInfo(const MeshUnwrapper *unwrapper, int mesh_index, int& mesh_vert, int& num_triangle);
MESHLIB_API bool GetAtlasMeshData(const MeshUnwrapper *unwrapper, int mesh_index, float *positions, float *uvs, int num_vert, int *indices, int num_triangle);
MESHLIB_API void DestroyUnwrapper(const MeshUnwrapper *unwapper);



//} // namespace

