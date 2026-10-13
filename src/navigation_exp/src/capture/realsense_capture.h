#include <librealsense2/rs.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include "common/types.h"
#include <optional>
#include <atomic>
#include <memory>
#include <stdexcept>

#ifndef REALSENSECAPTURE_H
#define REALSENSECAPTURE_H

#define REALSENSE_ANGLE_FROM_HORIZONTAL 40.0f

#ifndef DEPTH_WIDTH
#define DEPTH_WIDTH 212
#endif
#ifndef DEPTH_HEIGHT
#define DEPTH_HEIGHT 120
#endif
#define DEPTH_PIXELS (DEPTH_WIDTH * DEPTH_HEIGHT)

const float angle = ((-180.0f + REALSENSE_ANGLE_FROM_HORIZONTAL) * static_cast<float>(M_PI)) / 180.0f;
DeprojParams params {
    .cos_a = std::cos(angle),
    .sin_a = std::sin(angle)
};

// Functions
void save_to_ply(const std::vector<Vertex> &vertices, const std::string &filename);
void save_matrices_to_txt(const std::vector<std::vector<float>> &heights,
                          const std::vector<std::vector<Coordinate>> &actualCoordinates,
                          const std::string &filename);
std::shared_ptr<Matrices> load_matrices_from_txt(const std::string &filename);
void capture_depth_matrix(Matrices &matrices, int decimationKernelSize, rs2::pipeline& pipe);
void deproject_cpu(const uint16_t *depth,
                          const DeprojParams &p,
                          float *out_x,
                          float *out_y,
                          float *out_z,
                          uint8_t *out_valid);

#endif
