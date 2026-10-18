#include <iostream>
#include "obstacle_detection/gradient_map.h"
#include "capture/realsense_capture.h"
#include "capture/extract_capture.h"
#include <chrono>
#include <cassert>
#include "obstacle_detection/obstacle_clustering_tree.h"

static Matrices matrices;
static float gradients[DEPTH_PIXELS];

int main()
{
    xrt::device device(0);
    auto uuid = device.load_xclbin("/lib/firmware/xilinx/<app>/gradient.xclbin");
    xrt::kernel gradient_krn(device, uuid, "gradient_kernel");
    matrices.boHeights = xrt::bo(device, DEPTH_PIXELS * sizeof(float), xrt::bo::flags::cacheable, gradient_krn.group_id(0));
    matrices.boX = xrt::bo(device, DEPTH_PIXELS * sizeof(float), xrt::bo::flags::cacheable, gradient_krn.group_id(1));
    matrices.boY = xrt::bo(device, DEPTH_PIXELS * sizeof(float), xrt::bo::flags::cacheable, gradient_krn.group_id(2));
    matrices.boValid = xrt::bo(device, DEPTH_PIXELS * sizeof(uint8_t), xrt::bo::flags::cacheable, gradient_krn.group_id(3));
    matrices.boGradients = xrt::bo(device, DEPTH_PIXELS * sizeof(float), xrt::bo::flags::cacheable, gradient_krn.group_id(4));
    matrices.heights = matrices.boHeights.map<float *>();
    matrices.coordX = matrices.boX.map<float *>();
    matrices.coordY = matrices.boY.map<float *>();
    matrices.valid = matrices.boValid.map<uint8_t *>();
    matrices.gradients = matrices.boGradients.map<float *>();

    std::vector<Vertex> vertices;
    if (!runMatrixCollector(matrices, vertices, 4))
        return 1;
    std::vector<Vertex> obstacleVertices;
    ObstacleClusteringTree obstacleTree(2);
    ParallelGradientCalculator::calculateGradientsParallel(matrices, gradient_krn, obstacleVertices, obstacleTree);

    std::vector<Vertex> gradientVertices;
    for (int i = 0; i < DEPTH_PIXELS; i++)
    {
        if (matrices.valid[i])
        {
            gradientVertices.push_back(Vertex(matrices.coordX[i], matrices.coordY[i], matrices.gradients[i]));
        }
    }

    save_to_ply(vertices, "vertices.ply");
    save_to_ply(gradientVertices, "gradients.ply");
    save_to_ply(obstacleVertices, "obstacles.ply");

    for (int run = 0; run < 5; run++)
    {
        std::vector<Vertex> runVertices;
        ObstacleClusteringTree runTree(2);
        auto start = std::chrono::high_resolution_clock::now();
        ParallelGradientCalculator::calculateGradientsParallel(matrices, gradient_krn, runVertices, runTree);
        std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
        std::cout << "FPGA run " << run << " runtime: " << elapsed.count() << "s" << std::endl;
        assert(elapsed.count() > 0 && elapsed.count() < 0.04);
    }

    return 0;
}
