#pragma once

struct Options {
    int minClusterCount = 1;
    float maxChartArea = 1000.0f;
    float normalWeight = 1.0f;
    int maxIterationNum = 100;
};


extern "C" __declspec(dllexport) int MeshSegmentation(Options options, const float vertices[], int numVertices, const int indices[], int triangleNum, int clusterIds[]);