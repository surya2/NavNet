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

// class VisualizationSocket
// {
// private:
//     socket_t sockfd;
//     struct sockaddr_in serv_addr;
//     bool connected;

// public:
//     VisualizationSocket(const std::string &ip = "127.0.0.1", int port = 12345) : connected(false)
//     {
//         WSADATA wsaData;
//         if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
//         {
//             std::cerr << "WSAStartup failed" << std::endl;
//             return;
//         }
//         // Create socket
//         sockfd = socket(AF_INET, SOCK_STREAM, 0);
//         if (sockfd < 0)
//         {
//             std::cerr << "ERROR opening socket" << std::endl;
//             return;
//         }

//         // Set up server address
//         memset(&serv_addr, 0, sizeof(serv_addr));
//         serv_addr.sin_family = AF_INET;
//         serv_addr.sin_port = htons(port);

//         if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0)
//         {
//             std::cerr << "Invalid address or address not supported" << std::endl;
//             close(sockfd);
//             return;
//         }

//         if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
//         {
//             std::cerr << "Connection Failed. Python visualization server might not be running." << std::endl;
//             close(sockfd);
//             return;
//         }

//         connected = true;
//         std::cout << "Connected to visualization server" << std::endl;
//     }

//     ~VisualizationSocket()
//     {
//         if (connected)
//         {
//             sendMessage("TERMINATE");
//             close(sockfd);
//         }
//     }

//     void sendMessage(const std::string &message)
//     {
//         if (!connected)
//             return;

//         send(sockfd, message.c_str(), message.length(), 0);
//     }

//     void sendExploredPoint(float currentX, float currentY, float parentX, float parentY)
//     {
//         if (!connected)
//             return;

//         std::stringstream ss;
//         ss << "EXPLORE," << currentX << "," << currentY << "," << parentX << "," << parentY;
//         sendMessage(ss.str());
//     }

//     void sendTwist(float linVelocity, float angVelocity, int order)
//     {
//         if (!connected)
//             return;

//         std::stringstream ss;
//         ss << "TWIST," << linVelocity << "," << angVelocity << "," << order;
//         sendMessage(ss.str());
//     }

//     void sendPathPoint(float x, float y)
//     {
//         if (!connected)
//             return;

//         std::stringstream ss;
//         ss << "PATH," << x << "," << y;
//         sendMessage(ss.str());
//     }

//     bool isConnected() const
//     {
//         return connected;
//     }
// };

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
    //VisualizationSocket vizSocket;
    std::vector<Vertex> path;
    PathPoint *prev = nullptr;
    float prev_heading = 0.0f;
    while (current != nullptr)
    {
        path.push_back(Vertex(current->x, current->y, m.heights[Matrices::idx(current->indices.second, current->indices.first)]));
        if (prev != nullptr)
        {
            // Calculate heading to current point
            float dx = current->x - prev->x;
            float dy = current->y - prev->y;
            float target_heading = atan2(dy, dx);

            // Calculate angular difference (need to handle wrap-around)
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
            // if (vizSocket.isConnected())
            // {
            //     vizSocket.sendTwist(linear_velocity, angular_velocity, twists.size());
            //     usleep(25000); // 25ms delay
            // }

            // Update previous heading for next iteration
            prev_heading = target_heading;
        }
        // if (vizSocket.isConnected())
        // {
        //     vizSocket.sendPathPoint(current->x, current->y);
        //     // vizSocket.sendTwist(linear_velocity, angular_velocity, twists.size());
        //     usleep(50000); // 50ms delay
        // }
        prev = current;
        current = current->parent;
    }
    return path;
}

bool AStarPathPlanner::isDestination(PathPoint *current)
{
    return fabs(current->x - goal.x) <= 0.1 && fabs(current->y - goal.y) <= 0.1;
}

std::vector<Vertex> AStarPathPlanner::planPath(const Matrices &m,
                                               const float *gradients,
                                               Vertex &start,
                                               std::pair<int, int> &startIndices)
{
    constexpr int rows = DEPTH_HEIGHT;
    constexpr int cols = DEPTH_WIDTH;

    std::cout << "\n=== Debug Information ===\n";
    std::cout << "Matrix dimensions: " << rows << "x" << cols << std::endl;
    std::cout << "Start indices: (" << startIndices.first << "," << startIndices.second << ")" << std::endl;
    std::cout << "Start coordinates: (" << start.x << "," << start.y << ")" << std::endl;
    std::cout << "Goal coordinates: (" << goal.x << "," << goal.y << ")" << std::endl;

    constexpr float INVALID_POINT_PENALTY = 5.0f;       // Penalty for invalid points
    constexpr float OBSTACLE_PROXIMITY_PENALTY = 10.0f; // Penalty for being near obstacles
    constexpr float GRADIENT_COST_WEIGHT = 1.0f;

    // VisualizationSocket vizSocket;

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
        std::cout << "Error: Start indices out of bounds!" << std::endl;
        return std::vector<Vertex>();
    }

    const bool startValid = m.valid[startIndex];
    std::cout << "Start coordinate valid: " << (startValid ? "true" : "false") << std::endl;
    std::cout << "Start coordinate actual position: (" << m.coordX[startIndex] << "," << m.coordY[startIndex] << ")" << std::endl;

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

    int iterations = 0;
    while (!openSet.empty())
    {
        iterations++;

        int currentIndex = openSet.top().first;
        openSet.pop();

        PathPoint *current = costArray[currentIndex];
        if (!current)
        {
            std::cout << "Error: Null current point at index " << currentIndex << std::endl;
            continue;
        }

        if (isDestination(current))
        {
            std::cout << "Path found after " << iterations << " iterations!" << std::endl;
            std::vector<Vertex> path = retracePath(current, m);
            for (auto &point : costArray)
            {
                delete point;
                point = nullptr;
            }
            std::cout << "Final OpenSet size: " << openSet.size() << std::endl;
            std::cout << "path size: " << path.size() << std::endl;
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

                Vertex v(nextX, nextY, 0);
                auto obstacle = obstacleTree.findNearestObstacle(v);
                if (obstacle)
                {
                    float obstacle_dist = static_cast<float>(sqrt(
                        pow(obstacle->getVertex().x - nextX, 2) +
                        pow(obstacle->getVertex().y - nextY, 2)));
                    if (obstacle_dist < 0.2)
                    {
                        continue;
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
                localCost += GRADIENT_COST_WEIGHT * std::abs(gradients[neighborIndex]);
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
                    // if (vizSocket.isConnected())
                    // {
                    //     vizSocket.sendExploredPoint(
                    //         nextX, nextY,
                    //         current->x, current->y);
                    // }
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
                    // if (vizSocket.isConnected() && oldParent != current)
                    // {
                    //     vizSocket.sendExploredPoint(
                    //         nextX, nextY,
                    //         current->x, current->y);
                    // }
                }
            }
        }
    }

    // std::cout << "No path found after " << iterations << " iterations" << std::endl;
    // std::cout << "Final OpenSet size: " << openSet.size() << std::endl;
    for (auto &point : costArray)
    {
        delete point;
        point = nullptr;
    }
    return std::vector<Vertex>();
}