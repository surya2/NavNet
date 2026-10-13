#pragma once
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <thread>
#include <mutex>

#define M_PI 3.14159265358979323846

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

struct Coordinate
{
    float x, y;
    bool valid = false;
    Coordinate(float x, float y) : x(x), y(y) { valid = true; }
    Coordinate() {}
    Coordinate &operator=(const Coordinate &other)
    {
        if (this == &other)
            return *this;
        x = other.x;
        y = other.y;
        valid = other.valid;
        return *this;
    }
};

struct Matrices
{
    std::vector<std::vector<float>> heights;
    std::vector<std::vector<Coordinate>> actualCoordinates;
};
