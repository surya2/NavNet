#include <iostream>
#include "obstacle_detection/gradient_map.h"
#include "capture/realsense_capture.h"
#include "capture/extract_capture.h"
#include "obstacle_detection/obstacle_clustering_tree.h"
#include <chrono>
#include <cassert>

static Matrices matrices;

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

    Vertex v(-0.07, 0.5, -0.2455);
    auto obstacle = obstacleTree.findNearestObstacle(v);
    bool isNearObstacle = false;
    if (obstacle != nullptr)
    {
        if (sqrt(pow(obstacle->getVertex().x - (-0.07), 2) + pow(obstacle->getVertex().y - 0.5, 2)) < 0.05)
            isNearObstacle = true;
    }

    std::cout << "Is near obstacle: " << isNearObstacle << std::endl;

    v = Vertex(0.164, 0.614, 0);
    auto obstacle1 = obstacleTree.findNearestObstacle(v);
    isNearObstacle = false;
    if (obstacle1 != nullptr)
    {
        std::cout << "Obstacle 1: " << obstacle1->getVertex().x << ", " << obstacle1->getVertex().y << ", " << obstacle1->getVertex().z << std::endl;
        if (sqrt(pow(obstacle1->getVertex().x - (0.164), 2) + pow(obstacle1->getVertex().y - 0.614, 2)) < 0.1)
            isNearObstacle = true;
    }

    std::cout << "Is near obstacle: " << isNearObstacle << std::endl;

    v = Vertex(-0.494, 0.5, 0);
    auto obstacle2 = obstacleTree.findNearestObstacle(v);
    isNearObstacle = false;
    if (obstacle2 != nullptr)
    {
        if (sqrt(pow(obstacle2->getVertex().x - (-0.494), 2) + pow(obstacle2->getVertex().y - 0.5, 2)) < 0.1)
            isNearObstacle = true;
    }

    std::cout << "Is near obstacle: " << isNearObstacle << std::endl;

    // std::string filename = "../../../test/assets/matrix_benchmarks/matrix_benchmark_2025-01-30-21-15-38";
    // std::shared_ptr<Matrices> retMatrices = load_matrices_from_txt(filename + ".txt");

    // std::vector<Vertex> obstacleVertices2;
    // ObstacleClusteringTree obstacleTree2(2);
    // std::cout << "retMatrices->heights.size(): " << retMatrices->heights.size() << std::endl;
    // std::cout << "retMatrices->heights[0].size(): " << retMatrices->heights[0].size() << std::endl;
    // std::cout << "retMatrices->actualCoordinates.size(): " << retMatrices->actualCoordinates.size() << std::endl;
    // std::cout << "retMatrices->actualCoordinates[0].size(): " << retMatrices->actualCoordinates[0].size() << std::endl;
    // std::cout << "obstacleVertices2.size(): " << obstacleVertices2.size() << std::endl;
    // std::vector<std::vector<float>> gradients2 = ParallelGradientCalculator::calculateGradientsParallel(retMatrices->heights, retMatrices->actualCoordinates, 3, obstacleVertices2, obstacleTree2);
    // std::cout << "gradients2.size(): " << gradients2.size() << std::endl;
    // std::cout << "gradients2[0].size(): " << gradients2[0].size() << std::endl;

    // std::string test_cases = "../../../test/assets/matrix_benchmarks/test_points.case.txt";
    // std::ifstream in(test_cases);
    // if (!in.is_open())
    // {
    //     std::cerr << "Error: Could not open file " << test_cases << std::endl;
    //     return 1;
    // }
    // std::string line;
    // bool first = true;
    // while (std::getline(in, line))
    // {
    //     if (first)
    //     {
    //         first = false;
    //         continue;
    //     }
    //     float x, y;
    //     int expected;
    //     const char *lineChar = line.c_str();
    //     sscanf(lineChar, "%f, %f, %d", &x, &y, &expected);
    //     std::cout << "Testing point: " << x << ", " << y << std::endl;
    //     auto obstacle = obstacleTree2.findNearestObstacle(Vertex(x, y, -0.2455));
    //     bool isNearObstacle = false;
    //     if (obstacle != nullptr)
    //     {
    //         if (sqrt(pow(obstacle->getVertex().x - x, 2) + pow(obstacle->getVertex().y - y, 2)) < 0.05)
    //             isNearObstacle = true;
    //         // assert(isNearObstacle == expected);
    //         std::cout << "Is near point: " << obstacle->getVertex().x << ", " << obstacle->getVertex().y << ", " << obstacle->getVertex().z << " but is " << isNearObstacle << std::endl;
    //     }
    // }

    return 0;
}