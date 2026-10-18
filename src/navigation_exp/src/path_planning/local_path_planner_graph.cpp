#include "path_planning/local_path_planner_graph.h"
#include <utility>
#include <vector>
#include <queue>
#include <map>
#include <unordered_map>
#include <utility>
#include <string>
#include <sstream>

#ifdef _WIN32
void usleep_simulation(unsigned int microseconds);
#define usleep usleep_simulation
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <stdio.h>
#pragma comment(lib, "Ws2_32.lib")
#define close closesocket
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

using socket_t = decltype(socket(0, 0, 0));

void usleep_simulation(unsigned int microseconds)
{
    volatile unsigned long long i;
    for (i = 0; i < (unsigned long long)microseconds * 1000; i++)
    {
        // Do nothing, just burn CPU cycles
    }
}

struct Twist
{
    float linear;  // Linear velocity
    float angular; // Angular velocity

    Twist(float lin = 0.0f, float ang = 0.0f) : linear(lin), angular(ang) {}
};

float AStarPathPlanner::hValue(float x, float y)
{
    return static_cast<float>(sqrt(pow(x - goal.x, 2) + pow(y - goal.y, 2)));
}

struct CompareCoordinate
{
    bool operator()(std::pair<int, float> const &p1, std::pair<int, float> const &p2)
    {
        return p1.second > p2.second;
    }
};

std::vector<Vertex> AStarPathPlanner::retracePath(PathPoint *current, const Matrices &m)
{
    std::vector<Twist> twists;
    std::vector<Vertex> path;
    PathPoint *prev = nullptr;
    float prev_heading = 0.0f;
    while (current != nullptr)
    {
        path.push_back(Vertex(current->x, current->y, m.heights[Matrices::idx(current->indices.second, current->indices.first)]));
        if (prev != nullptr)
        {
            float dx = current->x - prev->x;
            float dy = current->y - prev->y;
            float target_heading = atan2(dy, dx);

            float angular_diff = target_heading - prev_heading;
            float angular_delta = static_cast<float>(2 * M_PI);
            if (angular_diff > M_PI)
                angular_diff -= angular_delta;
            if (angular_diff < -M_PI)
                angular_diff += angular_delta;

            // Calculate linear velocity (can be based on distance)
            float distance = sqrt(dx * dx + dy * dy);
            float linear_velocity = distance; // You might want to scale this

            // Calculate angular velocity
            float angular_velocity = angular_diff; // You might want to scale this

            // Add Twist to the vector
            twists.push_back(Twist(linear_velocity, angular_velocity));

            // Update previous heading for next iteration
            prev_heading = target_heading;
        }
        prev = current;
        current = current->parent;
    }
    return path;
}

std::vector<Vertex> AStarPathPlanner::mergePath(const std::vector<Vertex> &previous, std::vector<Vertex> &path, float x, float y)
{
    if (path.empty())
    {
        return previous;
    }
    size_t closest = previous.size();
    float best = INFINITY;
    for (size_t i = 0; i < previous.size(); i++)
    {
        const float d = (previous[i].x - x) * (previous[i].x - x) + (previous[i].y - y) * (previous[i].y - y);
        if (d < best)
        {
            best = d;
            closest = i;
        }
    }
    if (closest < previous.size())
    {
        path.insert(path.end(), previous.begin() + closest + 1, previous.end());
    }
    return path;
}

bool AStarPathPlanner::isDestination(PathPoint *current)
{
    return fabs(current->x - goal.x) <= 0.1 && fabs(current->y - goal.y) <= 0.1;
}

bool AStarPathPlanner::footprintCollides(float x, float y, float headingX, float headingY)
{
    constexpr float SIDE_CLEARANCE = 0.1f;
    const float halfLength = length / 2 + SIDE_CLEARANCE;
    const float halfSide = halfWidth + SIDE_CLEARANCE;
    Vertex center(x - headingX * length / 2, y - headingY * length / 2, 0);
    nearbyObstacles.clear();
    obstacleTree.findWithinRadius(center, std::sqrt(halfLength * halfLength + halfSide * halfSide), nearbyObstacles);
    for (const Vertex &o : nearbyObstacles)
    {
        const float dx = o.x - center.x;
        const float dy = o.y - center.y;
        const float forward = dx * headingX + dy * headingY;
        const float right = dx * headingY - dy * headingX;
        if (std::fabs(forward) <= halfLength && std::fabs(right) <= halfSide)
        {
            return true;
        }
    }
    return false;
}

std::vector<Vertex> AStarPathPlanner::planPath(const Matrices &m,
                                               Vertex &start,
                                               std::pair<int, int> &startIndices)
{
    constexpr int rows = DEPTH_HEIGHT;
    constexpr int cols = DEPTH_WIDTH;

    #ifdef PATH_PLANNER_DEBUG
    std::cout << "\n=== Path Planning Debug Information ===\n";
    std::cout << "Matrix dimensions: " << rows << "x" << cols << std::endl;
    std::cout << "Start indices: (" << startIndices.first << "," << startIndices.second << ")" << std::endl;
    std::cout << "Start coordinates: (" << start.x << "," << start.y << ")" << std::endl;
    std::cout << "Goal coordinates: (" << goal.x << "," << goal.y << ")" << std::endl;
    #endif

    constexpr float INVALID_POINT_PENALTY = 5.0f;       // Penalty for invalid points
    constexpr float OBSTACLE_PROXIMITY_PENALTY = 10.0f; // Penalty for being near obstacles
    constexpr float GRADIENT_COST_WEIGHT = 1.0f;

    for (auto &point : costArray)
    {
        delete point;
        point = nullptr;
    }

    std::priority_queue<std::pair<int, float>,
                        std::vector<std::pair<int, float>>,
                        CompareCoordinate>
        openSet;

    int startIndex = startIndices.first * cols + startIndices.second;

    if (startIndices.first >= rows ||
        startIndices.second >= cols)
    {
        #ifdef PATH_PLANNER_DEBUG
        std::cout << "Error: Start indices out of bounds!" << std::endl;
        #endif
        return std::vector<Vertex>();
    }

    const bool startValid = m.valid[startIndex];
    #ifdef PATH_PLANNER_DEBUG
    std::cout << "Start coordinate valid: " << (startValid ? "true" : "false") << std::endl;
    std::cout << "Start coordinate actual position: (" << m.coordX[startIndex] << "," << m.coordY[startIndex] << ")" << std::endl;
    #endif

    float startCost = startValid ? 0.0f : INVALID_POINT_PENALTY;
    PathPoint *startNode = new PathPoint(nullptr,
                                         m.coordX[startIndex],
                                         m.coordY[startIndex],
                                         startIndices.first,
                                         startIndices.second,
                                         startCost,
                                         startCost);

    costArray[startIndex] = startNode;
    openSet.push({startIndex, startCost});

    std::unordered_map<int, bool> closedSet;

    constexpr int MAX_PLAN_ITERATIONS = 5000;
    PathPoint *bestNode = startNode;
    float bestH = hValue(startNode->x, startNode->y);

    int iterations = 0;
    while (!openSet.empty() && iterations < MAX_PLAN_ITERATIONS)
    {
        iterations++;

        int currentIndex = openSet.top().first;
        openSet.pop();

        PathPoint *current = costArray[currentIndex];
        if (!current)
        {
            #ifdef PATH_PLANNER_DEBUG
            std::cout << "Error: Null current point at index " << currentIndex << std::endl;
            #endif
            continue;
        }

        if (m.valid[currentIndex] && hValue(current->x, current->y) < bestH)
        {
            bestH = hValue(current->x, current->y);
            bestNode = current;
        }

        if (isDestination(current))
        {
            #ifdef PATH_PLANNER_DEBUG
            std::cout << "Path found after " << iterations << " iterations!" << std::endl;
            #endif
            std::vector<Vertex> path = retracePath(current, m);
            for (auto &point : costArray)
            {
                delete point;
                point = nullptr;
            }
            #ifdef PATH_PLANNER_DEBUG
            std::cout << "Final OpenSet size: " << openSet.size() << std::endl;
            std::cout << "path size: " << path.size() << std::endl;
            #endif
            return path;
        }

        if (closedSet.find(currentIndex) != closedSet.end())
        {
            continue;
        }

        closedSet[currentIndex] = true;

        int neighborsFound = 0;

#pragma omp parallel for collapse(2) shared(costArray, openSet)
        for (int dy = -2; dy <= 2; dy += 2)
        {
            for (int dx = -2; dx <= 2; dx += 2)
            {
                if (dx == 0 && dy == 0)
                    continue;

                int newRow = current->indices.first + dy;
                int newCol = current->indices.second + dx;

                if (newRow < 0 || newRow >= rows ||
                    newCol < 0 || newCol >= cols)
                {
                    continue;
                }

                const int neighborIndex = newRow * cols + newCol;
                const float nextX = m.coordX[neighborIndex];
                const float nextY = m.coordY[neighborIndex];
                const bool nextValid = m.valid[neighborIndex];
                float moveCost = 0.0; //  = (abs(dx) + abs(dy) == 2) ? 1.414f : 1.0f

                if (!nextValid)
                {
                    moveCost += INVALID_POINT_PENALTY;
                }

                if (nextValid)
                {
                    const bool currentValid = m.valid[current->indices.first * cols + current->indices.second];
                    float headingX = currentValid ? nextX - current->x : goal.x - nextX;
                    float headingY = currentValid ? nextY - current->y : goal.y - nextY;
                    const float headingLength = std::hypot(headingX, headingY);
                    if (headingLength > 1e-6f)
                    {
                        headingX /= headingLength;
                        headingY /= headingLength;
                        if (footprintCollides(nextX, nextY, headingX, headingY))
                        {
                            continue;
                        }
                    }
                }

                float localCost = current->costs.first;
                if (nextValid)
                {
                    localCost += static_cast<float>(sqrt(pow(nextX - current->x, 2) + pow(nextY - current->y, 2)));
                }
                else
                {
                    localCost += 1.0f;
                }
                localCost += GRADIENT_COST_WEIGHT * std::abs(m.gradients[neighborIndex]);
                float newCost = localCost + moveCost;
                PathPoint *neighbor = costArray[neighborIndex];
                if (!neighbor)
                {
                    neighborsFound++;
                    if (nextValid)
                    {
                        newCost += hValue(nextX, nextY);
                    }
                    neighbor = new PathPoint(current, nextX, nextY, newRow, newCol,
                                             localCost, newCost);
                    neighbor->parent = current;
                    costArray[neighborIndex] = neighbor;
#pragma omp critical
                    {
                        openSet.push({neighborIndex, newCost});
                    }
                }
                else if (newCost < neighbor->costs.first)
                {
                    neighborsFound++;
                    PathPoint *oldParent = neighbor->parent;
                    neighbor->parent = current;
                    neighbor->costs.first = localCost;
                    if (nextValid)
                    {
                        newCost += hValue(nextX, nextY);
                    }
                    neighbor->costs.second = newCost;
#pragma omp critical
                    {
                        openSet.push({neighborIndex, newCost});
                    }
                }
            }
        }
    }
    std::vector<Vertex> partial;
    if (bestNode != startNode)
    {
        partial = retracePath(bestNode, m);
    }
    for (auto &point : costArray)
    {
        delete point;
        point = nullptr;
    }
    return partial;
}