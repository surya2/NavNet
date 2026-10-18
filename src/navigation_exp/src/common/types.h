#pragma once
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#include <xrt/xrt_bo.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <thread>
#include <mutex>
#include <cstdint>

#ifndef DEPTH_WIDTH
#define DEPTH_WIDTH 212
#endif
#ifndef DEPTH_HEIGHT
#define DEPTH_HEIGHT 120
#endif
#define DEPTH_PIXELS (DEPTH_WIDTH * DEPTH_HEIGHT)

struct Matrices
{
    static constexpr int width = DEPTH_WIDTH;
    static constexpr int height = DEPTH_HEIGHT;

    xrt::bo boHeights;
    xrt::bo boX;
    xrt::bo boY;
    xrt::bo boValid;
    xrt::bo boGradients;

    float *heights;
    float *coordX;
    float *coordY;
    uint8_t *valid;
    float *gradients;

    static constexpr int idx(int x, int y) { return y * DEPTH_WIDTH + x; }
};

struct DeprojParams
{
    float fx, fy, ppx, ppy; // depth intrinsics
    float depth_units;      // meters per raw depth unit
    float cos_a, sin_a;     // rotation of the camera about the x axis
};