#include <librealsense2/rs.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include "common/types.h"
#include <optional>
#include <atomic>

#ifndef REALSENSECAPTURE_H
#define REALSENSECAPTURE_H

#define REALSENSE_ANGLE_FROM_HORIZONTAL 40.0f

void save_to_ply(const std::vector<Vertex> &vertices, const std::string &filename);
void save_matrices_to_txt(const std::vector<std::vector<float>> &heights,
                          const std::vector<std::vector<Coordinate>> &actualCoordinates,
                          const std::string &filename);
std::shared_ptr<Matrices> load_matrices_from_txt(const std::string &filename);
std::shared_ptr<Matrices> capture_depth_matrix(std::optional<std::vector<Vertex> *> &vertices, int decimationKernelSize, rs2::pipeline& pipe);

#endif
