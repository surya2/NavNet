#ifndef EXTRACTCAPTURE_H
#define EXTRACTCAPTURE_H

#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include "capture/realsense_capture.h"

std::shared_ptr<Matrices> runMatrixCollector(std::vector<Vertex> &vertices, int decimationKernelSize);

#endif
