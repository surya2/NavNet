#ifndef EXTRACTCAPTURE_H
#define EXTRACTCAPTURE_H

#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include "capture/realsense_capture.h"

// #define DEPTH_TIMING

bool runMatrixCollector(Matrices &matrices, std::vector<Vertex> &vertices, int decimationKernelSize);

#endif
