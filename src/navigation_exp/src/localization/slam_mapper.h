#ifndef SLAM_H
#define SLAM_H

#include <vector>
#include <cmath>
#include <utility>
#include "common/types.h"
#include "common/mathematics.h"
#include "obstacle_detection/obstacle_clustering_tree.h"

class SLAM_Graph
{
private:
    std::vector<Coordinate *> poses;
    std::vector<Coordinate *> landmarks;
    std::vector<Coordinate *> odom_edges; // Odometry edges
    std::vector<Coordinate *> loop_edges; // Loop closures

public:
    Coordinate currentRobotPos = Coordinate(0, 0, 0);
    Coordinate currentRobotVel = Coordinate(0, 0, 0);
}
#endif