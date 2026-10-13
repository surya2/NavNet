#include "capture/realsense_capture.h"
#include "obstacle_detection/gradient_map.h"
#include "path_planning/local_path_planner_graph.h"
#include <csignal>

#define DECIMATION_KERNEL_SIZE 4
#define GRADIENT_THREADS 3
#define GOAL_X 0.0f
#define GOAL_Y 1.0f

volatile std::sig_atomic_t halt = 0;
static Matrices matrices;

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        halt = 1;
    }
}

int main()
{
    std::signal(SIGINT, signal_handler);
    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

    pipe.start(cfg);

    for (int i = 0; i < 30; i++)
    {
        pipe.wait_for_frames();
    }
    while (!halt)
    {
        if (!capture_depth_matrix(matrices, DECIMATION_KERNEL_SIZE, pipe))
        {
            continue;
        }
        std::vector<Vertex> obstacleVertices;
        ObstacleClusteringTree obstacleTree(2);
        ParallelGradientCalculator::calculateGradientsParallel(matrices, GRADIENT_THREADS, obstacleVertices, obstacleTree);
        std::pair<int, int> startIndices = {DEPTH_HEIGHT - 1, DEPTH_WIDTH / 2};
        const int startIndex = Matrices::idx(startIndices.second, startIndices.first);
        Vertex start(matrices.coordX[startIndex], matrices.coordY[startIndex], 0);
        AStarPathPlanner planner(obstacleTree, Vertex(GOAL_X, GOAL_Y, 0));
        std::vector<Vertex> path = planner.planPath(matrices, start, startIndices);
    }
    pipe.stop();
    return 0;
}
