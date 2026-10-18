#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#include <xrt/xrt_bo.h>

#include "capture/realsense_capture.h"
#include "obstacle_detection/gradient_map.h"
#include "path_planning/local_path_planner_graph.h"
#include "communication/frame_sender.h"
#include <csignal>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

// #define STARTUP_DEBUG
#define DECIMATION_KERNEL_SIZE 4
#define GRADIENT_THREADS 3

volatile std::sig_atomic_t halt = 0;
static Matrices matrices;
static Motion motion;
static Vehicle vehicle_state;
// static std::vector<Vertex> fullPath;


void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        halt = 1;
    }
}

bool try_convert_to_float(std::string num_str, float *num){
    try {
        *num = std::stod(num_str);
        return true;
    } 
    catch (const std::invalid_argument& e) {
        std::cerr << "Not a valid a number" << std::endl;
        return false;
    }
}

int main(int argc, char **argv)
{
    // parse vehicle config first
    std::ifstream file("config/vehicle.conf");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the config file!" << std::endl;
        return 1;
    }
    std::string line;
    int line_number = 0;
    
    while (std::getline(file, line)) {
        line_number++;
        std::stringstream ss(line);
        std::string word;

        std::vector<std::string> words;

        while (ss >> word && word[0] != '#') {
            words.push_back(word);
        }

        if (words.empty()) continue;

        if(words.size() != 2){
            std::cerr << "Line does not contain exactly 2 words!" << std::endl;
            return 1;
        }

        std::string first_word = words[0];
        std::string second = words[1];

        if(first_word == "width"){
            if(!try_convert_to_float(second, &vehicle_state.width)){
                return 1;
            }
        }
        else if(first_word == "height"){
            if(!try_convert_to_float(second, &vehicle_state.height)){
                return 1;
            }
        }
        else if(first_word == "wheel_height"){
            if(!try_convert_to_float(second, &vehicle_state.wheel_height)){
                return 1;
            }
        }
        else if(first_word == "destination"){
            std::stringstream coord_stream(second);
            std::string dst_x_str, dst_y_str;

            if (std::getline(coord_stream, dst_x_str, ',') && std::getline(coord_stream, dst_y_str, ',')) {
                if(!try_convert_to_float(dst_x_str, &vehicle_state.destination.x)){
                    return 1;
                }
                if(!try_convert_to_float(dst_y_str, &vehicle_state.destination.y)){
                    return 1;
                }
            } else {
                std::cerr << "Coordinate format for destination is missing a comma on line " << std::endl;
            }
        }
    }
    file.close();


    // start capture to path planning sequence
    std::signal(SIGINT, signal_handler);

    xrt::device device(0);
    auto uuid = device.load_xclbin("/lib/firmware/xilinx/gradient_accel/binary_container_1.xclbin");
    xrt::kernel gradient_krn(device, uuid, "gradient_kernel");
    matrices.boHeights = xrt::bo(device, DEPTH_PIXELS * sizeof(float), xrt::bo::flags::cacheable, gradient_krn.group_id(0));
    matrices.boX = xrt::bo(device, DEPTH_PIXELS * sizeof(float), xrt::bo::flags::cacheable, gradient_krn.group_id(1));
    matrices.boY = xrt::bo(device, DEPTH_PIXELS * sizeof(float), xrt::bo::flags::cacheable, gradient_krn.group_id(2));
    matrices.boValid = xrt::bo(device, DEPTH_PIXELS * sizeof(uint8_t), xrt::bo::flags::cacheable, gradient_krn.group_id(3));
    matrices.boGradients = xrt::bo(device, DEPTH_PIXELS * sizeof(float), xrt::bo::flags::cacheable, gradient_krn.group_id(4));
    matrices.heights = matrices.boHeights.map<float *>();
    matrices.coordX = matrices.boX.map<float *>();
    matrices.coordY = matrices.boY.map<float *>();
    matrices.valid= matrices.boValid.map<uint8_t *>();
    matrices.gradients = matrices.boGradients.map<float *>();

    static FrameSender sender("10.0.0.14");
    auto start = std::chrono::steady_clock::now();
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 63);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
    try {
        pipe.start(cfg);
    } catch (const rs2::error &e) {
        std::cerr << "Pipeline start failed: " << e.what() << "\n";
        return false;
    }
    // warmup
    for (int i = 0; i < 15; i++) {
        rs2::frameset fs;
        if (!pipe.try_wait_for_frames(&fs, 1000)) break;
    }
    double warmup_elapsed_time = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    #ifdef STARTUP_DEBUG
    std::cout << "Warmup elapsed time in ms: " << warmup_elapsed_time << std::endl;
    #endif

    while (!halt)
    {
        double capture_elapsed;
        auto frame_start = std::chrono::steady_clock::now();
        if (!capture_depth_matrix(matrices, DECIMATION_KERNEL_SIZE, pipe, capture_elapsed, motion, vehicle_state, frame_start))
        {
            continue;
        }
        #ifdef STARTUP_DEBUG
        std::cout << "Capture elapsed time in ms: " << capture_elapsed << std::endl;
        #endif
        std::vector<Vertex> obstacleVertices;
        ObstacleClusteringTree obstacleTree(2);
        auto obstacles_start = std::chrono::steady_clock::now();
        ParallelGradientCalculator::calculateGradientsParallel(matrices, gradient_krn, obstacleVertices, obstacleTree);
        double obstacles_elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - obstacles_start).count();    
        #ifdef STARTUP_DEBUG
        std::cout << "Obstacles elapsed time in ms: " << obstacles_elapsed << std::endl;
        #endif

        std::pair<int, int> startIndices = {DEPTH_HEIGHT - 1, DEPTH_WIDTH / 2};
        const int startIndex = Matrices::idx(startIndices.second, startIndices.first);
        Vertex start(matrices.coordX[startIndex], matrices.coordY[startIndex], 0);
        AStarPathPlanner planner(obstacleTree, Vertex(vehicle_state.destination.x, vehicle_state.destination.y, 0), vehicle_state);
        auto path_start = std::chrono::steady_clock::now();
        std::vector<Vertex> path = planner.planPath(matrices, start, startIndices);
        // fullPath = AStarPathPlanner::mergePath(fullPath, path, vehicle_state.orientation.x, vehicle_state.orientation.y);
        auto end = std::chrono::steady_clock::now();
        double path_elapsed = std::chrono::duration<double, std::milli>(end - path_start).count();
        double elapsed = std::chrono::duration<double, std::milli>(end - frame_start).count();
        // sender.send(matrices, obstacleVertices, fullPath,elapsed, capture_elapsed, obstacles_elapsed, path_elapsed);
        sender.send(matrices, obstacleVertices, path, elapsed, capture_elapsed, obstacles_elapsed, path_elapsed);
    }
    pipe.stop();
    return 0;
}
