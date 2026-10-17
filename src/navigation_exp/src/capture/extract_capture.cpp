#include "capture/extract_capture.h"

bool runMatrixCollector(Matrices &matrices, std::vector<Vertex> &vertices, int decimationKernelSize)
{
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
    #ifdef DEPTH_TIMING
    std::cout << "Warmup elapsed time in ms: " << warmup_elapsed_time << std::endl;
    #endif

    Motion motion;
    double captureElapsed;
    if (!capture_depth_matrix(matrices, decimationKernelSize, pipe, captureElapsed, motion)){
        return false;
    }
    for (int i = 0; i < DEPTH_PIXELS; i++){
        if (matrices.valid[i]){
            vertices.push_back(Vertex(matrices.coordX[i], matrices.coordY[i], matrices.heights[i]));
        }
    }
    #ifdef DEPTH_TIMING
    std::cout << "Capture elapsed time in ms: " << captureElapsed << std::endl;
    std::cout << "Total elapsed time in ms: " << captureElapsed + warmup_elapsed_time << std::endl;
    #endif
    return true;
}
