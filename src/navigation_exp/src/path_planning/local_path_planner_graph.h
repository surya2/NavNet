#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <cmath>
#include <utility>
#include "common/types.h"
#include "common/mathematics.h"
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

public:
    AStarPathPlanner(ObstacleClusteringTree &tree, Vertex&& goal)
        : obstacleTree(tree), goal(goal) {}
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

    float hValue(float x, float y);

    bool isDestination(PathPoint *current);
};
#endif