#ifndef GRADIENTMAP_H
#define GRADIENTMAP_H

#include <cmath>
#include <cstdint>
#include <vector>
#include "common/types.h"
#include "common/mathematics.h"
#include "obstacle_detection/obstacle_clustering_tree.h"

#ifndef GRADIENT_MAX_THREADS
#define GRADIENT_MAX_THREADS 16
#endif

#define GRADIENT_TIMING

struct Stats
{
    int validPoints;
    float mean;
    float stdDev;
    float gradMean;
    float gradStdDev;
    float localSumSquares;
    float localGradSumSquares;
};

struct GradientPartialSums
{
    long long count = 0;
    double sum_heights = 0.0;
    double sum_heights_squared = 0.0;
    double sum_gradients = 0.0;
    double sum_gradients_squared = 0.0;
};

float magnitude(float x, float y);

class ParallelGradientCalculator
{
public:
    static void calculateGradientsParallel(
        const Matrices &m,
        int numThreads,
        std::vector<Vertex> &obstacleVertices,
        ObstacleClusteringTree &obstacleTree,
        float *gradients);

private:
    // Raw-pointer compute functions: no std::vector, no heap, no shared_ptr.
    // These are the pieces that can later be turned into FPGA kernels.
    static void computeGradientRows(
        const float *heights,
        const float *coordX,
        const float *coordY,
        const uint8_t *valid,
        float *gradients,
        int rowBegin,
        int rowEnd,
        GradientPartialSums &sums);

    static float calculatePartialX(
        const float *heights,
        const float *coordX,
        const uint8_t *valid,
        int row, int col);

    static float calculatePartialY(
        const float *heights,
        const float *coordY,
        const uint8_t *valid,
        int row, int col);
};

#endif