#include <librealsense2/rs.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include "common/types.h"
#include "common/mathematics.h"
#include <optional>
#include <atomic>
#include <memory>
#include <stdexcept>

#ifndef REALSENSECAPTURE_H
#define REALSENSECAPTURE_H

#define REALSENSE_ANGLE_FROM_HORIZONTAL 40.0f

// Functions
void save_to_ply(const std::vector<Vertex> &vertices, const std::string &filename);
void save_matrices_to_txt(const Matrices &matrices, const std::string &filename);
bool load_matrices_from_txt(const std::string &filename, Matrices &matrices);
bool capture_depth_matrix(Matrices &matrices, int decimationKernelSize, rs2::pipeline& pipe, 
    double &elapaed_time, 
    Motion &motion);
void deproject_cpu(const uint16_t *depth, const DeprojParams &p,float *out_x, float *out_y,
    float *out_z,
    uint8_t *out_valid,
    Motion &motion);

#endif
