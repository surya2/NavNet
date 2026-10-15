#include "capture/realsense_capture.h"
#include "obstacle_detection/gradient_map.h"
#include "path_planning/local_path_planner_graph.h"
#include "communication/frame_sender.h"
#include <csignal>
#include <chrono>

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

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);
    static FrameSender sender(argc > 1 ? argv[1] : "255.255.255.255");
    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8);

    pipe.start(cfg);

    for (int i = 0; i < 30; i++)
    {
        pipe.wait_for_frames();
    }
    while (!halt)
    {
        double capture_elapsed;
        auto frame_start = std::chrono::steady_clock::now();
        if (!capture_depth_matrix(matrices, DECIMATION_KERNEL_SIZE, pipe, capture_elapsed))
        {
            continue;
        }
        std::vector<Vertex> obstacleVertices;
        ObstacleClusteringTree obstacleTree(2);
        auto obstacles_start = std::chrono::steady_clock::now();
        ParallelGradientCalculator::calculateGradientsParallel(matrices, GRADIENT_THREADS, obstacleVertices, obstacleTree);
        double obstacles_elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - obstacles_start).count();

        std::pair<int, int> startIndices = {DEPTH_HEIGHT - 1, DEPTH_WIDTH / 2};
        const int startIndex = Matrices::idx(startIndices.second, startIndices.first);
        Vertex start(matrices.coordX[startIndex], matrices.coordY[startIndex], 0);
        AStarPathPlanner planner(obstacleTree, Vertex(GOAL_X, GOAL_Y, 0));
        auto path_start = std::chrono::steady_clock::now();
        std::vector<Vertex> path = planner.planPath(matrices, start, startIndices);
        auto end = std::chrono::steady_clock::now();
        double path_elapsed = std::chrono::duration<double, std::milli>(end - path_start).count();
        double elapsed = std::chrono::duration<double, std::milli>(end - frame_start).count();
        sender.send(matrices, obstacleVertices, path, elapsed, capture_elapsed, obstacles_elapsed, path_elapsed);
    }
    pipe.stop();
    return 0;
}
