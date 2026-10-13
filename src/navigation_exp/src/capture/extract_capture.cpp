#include "capture/extract_capture.h"

bool runMatrixCollector(Matrices &matrices, std::vector<Vertex> &vertices, int decimationKernelSize)
{
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8);
    pipe.start(cfg);
    for (int i = 0; i < 30; i++)
        pipe.wait_for_frames();
    if (!capture_depth_matrix(matrices, decimationKernelSize, pipe))
        return false;
    for (int i = 0; i < DEPTH_PIXELS; i++)
        if (matrices.valid[i])
            vertices.push_back(Vertex(matrices.coordX[i], matrices.coordY[i], matrices.heights[i]));
    return true;
}
