#pragma once

struct Options {
    int minClusterCount = 1;
    float maxChartArea = 1000.0f;
    float normalWeight = 1.0f;
    int maxIterationCount = 100;
};


bool Segmentation(Options options, float *vertices, int numVertices, int *indices, int triangleNum, int *clusterIds);
