#pragma once
#include "common/types.h"
#include "common/mathematics.h"
#include <netinet/in.h>
#include <string>

enum PacketType : uint8_t
{
    POINT_CLOUD = 0,
    OBSTACLES = 1,
    PATH = 2,
    STATS = 3
};

#pragma pack(push, 1)
struct PacketHeader
{
    uint8_t type;
    uint8_t frameId;
    uint16_t chunkIndex;
    uint16_t chunkCount;
    uint16_t pointCount;
    uint16_t payloadSize;
    float min[3];
    float max[3];
    uint32_t crc;
};
#pragma pack(pop)
static_assert(sizeof(PacketHeader) == 38, "PacketHeader must be packed");

constexpr int CHUNK_POINTS = 200;

class FrameSender
{
    int sock;
    sockaddr_in dest{};
    uint8_t frameId = 0;
    float points[DEPTH_PIXELS * 3];
    int pointCount = 0;

    void sendList(PacketType type);
    void transmit(PacketHeader &header, uint8_t *packet);

public:
    explicit FrameSender(const std::string &ip, uint16_t port = 2003);
    ~FrameSender();
    FrameSender(const FrameSender &) = delete;
    FrameSender &operator=(const FrameSender &) = delete;
    void send(const Matrices &m, const std::vector<Vertex> &obstacles, const std::vector<Vertex> &path,
              double elapsed, double captureElapsed, double obstaclesElapsed, double pathElapsed);
};
