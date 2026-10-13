#pragma once
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <thread>
#include <mutex>
#include <cstdint>

#define M_PI 3.14159265358979323846

#ifndef DEPTH_WIDTH
#define DEPTH_WIDTH 212
#endif
#ifndef DEPTH_HEIGHT
#define DEPTH_HEIGHT 120
#endif
#define DEPTH_PIXELS (DEPTH_WIDTH * DEPTH_HEIGHT)

struct Vertex
{
    float x, y, z;
    Vertex() : x(-1), y(-1), z(-1) {}
    Vertex(float x, float y, float z) : x(x), y(y), z(z) {}

    bool operator==(const Vertex &other) const
    {
        return std::fabs(x - other.x) <= 0.5 && std::fabs(y - other.y) <= 0.5 && std::fabs(z - other.z) <= 0.5;
    }
};

struct Matrices
{
    static constexpr int width = DEPTH_WIDTH;
    static constexpr int height = DEPTH_HEIGHT;

    float heights[DEPTH_PIXELS] = {};
    float coordX[DEPTH_PIXELS] = {};
    float coordY[DEPTH_PIXELS] = {};
    uint8_t valid[DEPTH_PIXELS] = {};

    static constexpr int idx(int x, int y) { return y * DEPTH_WIDTH + x; }
};

struct DeprojParams
{
    float fx, fy, ppx, ppy; // depth intrinsics
    float depth_units;      // meters per raw depth unit
    float cos_a, sin_a;     // rotation of the camera about the x axis
};
