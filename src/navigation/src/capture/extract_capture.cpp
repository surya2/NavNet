#include "capture/extract_capture.h"

std::shared_ptr<Matrices> runMatrixCollector(std::vector<Vertex> &vertices, int decimationKernelSize)
{
    rs2::pipeline pipe;
    pipe.start();
    for (int i = 0; i < 30; i++)
        pipe.wait_for_frames();
    std::optional<std::vector<Vertex> *> v = &vertices;
    std::shared_ptr<Matrices> matrices = capture_depth_matrix(v, decimationKernelSize, pipe);
    return matrices;
}
