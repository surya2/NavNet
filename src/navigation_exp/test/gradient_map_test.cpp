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
    std::vector<Vertex> vertices;
    if (!runMatrixCollector(matrices, vertices, 4))
        return 1;
    std::vector<Vertex> obstacleVertices;
    ObstacleClusteringTree obstacleTree(2);
    ParallelGradientCalculator::calculateGradientsParallel(matrices, 3, obstacleVertices, obstacleTree, gradients);

    std::vector<Vertex> gradientVertices;
    for (int i = 0; i < DEPTH_PIXELS; i++)
    {
        if (matrices.valid[i])
        {
            gradientVertices.push_back(Vertex(matrices.coordX[i], matrices.coordY[i], gradients[i]));
        }
    }

    save_to_ply(vertices, "vertices.ply");
    save_to_ply(gradientVertices, "gradients.ply");
    save_to_ply(obstacleVertices, "obstacles.ply");

    /* Measure runtime of single thread :*/
    auto start = std::chrono::high_resolution_clock::now();
    ParallelGradientCalculator::calculateGradientsParallel(matrices, 1, obstacleVertices, obstacleTree, gradients);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Single thread runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 1);

    start = std::chrono::high_resolution_clock::now();
    ParallelGradientCalculator::calculateGradientsParallel(matrices, 2, obstacleVertices, obstacleTree, gradients);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "2 threads runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 0.04);

    start = std::chrono::high_resolution_clock::now();
    ParallelGradientCalculator::calculateGradientsParallel(matrices, 3, obstacleVertices, obstacleTree, gradients);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "3 threads runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 0.04);

    start = std::chrono::high_resolution_clock::now();
    ParallelGradientCalculator::calculateGradientsParallel(matrices, 4, obstacleVertices, obstacleTree, gradients);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "4 threads runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 0.04);

    start = std::chrono::high_resolution_clock::now();
    ParallelGradientCalculator::calculateGradientsParallel(matrices, 5, obstacleVertices, obstacleTree, gradients);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "5 threads runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 0.04);

    start = std::chrono::high_resolution_clock::now();
    ParallelGradientCalculator::calculateGradientsParallel(matrices, 6, obstacleVertices, obstacleTree, gradients);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "6 threads runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 0.04);

    return 0;
}