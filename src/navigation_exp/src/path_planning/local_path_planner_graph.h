#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <cmath>
#include <utility>
#include "common/types.h"
#include "common/mathematics.h"
#include "common/vehicle.h"
#include "obstacle_detection/obstacle_clustering_tree.h"

// #define PATH_PLANNER_DEBUG

struct PathPoint
{
    std::pair<int, int> indices;
    PathPoint *parent;
    float x, y;
    std::pair<float, float> costs; // local cost, global cost
    PathPoint(PathPoint *parent, float x, float y, int i, int j, float localCost, float globalCost) : indices({i, j}), parent(parent),
                                                                                                      x(x), y(y), costs({localCost, globalCost}) {}
};

class AStarPathPlanner
{
private:
    ObstacleClusteringTree &obstacleTree;
    PathPoint *costArray[DEPTH_PIXELS] = {};
    Vertex goal;
    float halfWidth;
    float length;
    std::vector<Vertex> nearbyObstacles;

public:
    AStarPathPlanner(ObstacleClusteringTree &tree, Vertex&& goal, const Vehicle &vehicle)
        : obstacleTree(tree), goal(goal), halfWidth(vehicle.width / 2), length(vehicle.height) {}
    AStarPathPlanner(const AStarPathPlanner &) = delete;
    AStarPathPlanner &operator=(const AStarPathPlanner &) = delete;
    ~AStarPathPlanner()
    {
        for (auto point : costArray)
        {
            delete point;
        }
    }

    std::vector<Vertex> retracePath(PathPoint *current, const Matrices &m);

    std::vector<Vertex> planPath(const Matrices &m, Vertex &start, std::pair<int, int> &startIndices);

    static std::vector<Vertex> mergePath(const std::vector<Vertex> &previous, std::vector<Vertex> &path, float x, float y);

    float hValue(float x, float y);

    bool isDestination(PathPoint *current);

    bool footprintCollides(float x, float y, float headingX, float headingY);
};
#endif