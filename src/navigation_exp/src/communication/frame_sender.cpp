#include "communication/frame_sender.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <lz4.h>
#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

static uint32_t crc32(const uint8_t *data, size_t len)
{
    static uint32_t table[256] = {};
    if (!table[1])
        for (uint32_t i = 0; i < 256; i++)
        {
            uint32_t c = i;
            for (int k = 0; k < 8; k++)
                c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++)
        crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

FrameSender::FrameSender(const std::string &ip, uint16_t port) : sock(socket(AF_INET, SOCK_DGRAM, 0))
{
    int on = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
    int sendBuffer = 4 * 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sendBuffer, sizeof(sendBuffer));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &dest.sin_addr) != 1)
        throw std::runtime_error("Invalid send address: " + ip);
}

FrameSender::~FrameSender()
{
    close(sock);
}

void FrameSender::send(const Matrices &m, const std::vector<Vertex> &obstacles, const std::vector<Vertex> &path,
                       double elapsed, double captureElapsed, double obstaclesElapsed, double pathElapsed)
{
    const float stats[4] = {static_cast<float>(elapsed), static_cast<float>(captureElapsed),
                            static_cast<float>(obstaclesElapsed), static_cast<float>(pathElapsed)};
    PacketHeader header{};
    header.type = STATS;
    header.frameId = frameId;
    header.chunkCount = 1;
    header.payloadSize = sizeof(stats);
    uint8_t packet[sizeof(PacketHeader) + sizeof(stats)];
    std::memcpy(packet + sizeof(PacketHeader), stats, sizeof(stats));
    transmit(header, packet);

    pointCount = 0;
    for (int i = 0; i < DEPTH_PIXELS; i++)
        if (m.valid[i])
        {
            float *p = &points[pointCount++ * 3];
            p[0] = m.heights[i];
            p[1] = m.coordX[i];
            p[2] = m.coordY[i];
        }
    sendList(POINT_CLOUD);

    for (auto [type, list] : {std::make_pair(OBSTACLES, &obstacles), std::make_pair(PATH, &path)})
    {
        pointCount = std::min(static_cast<int>(list->size()), DEPTH_PIXELS);
        for (int i = 0; i < pointCount; i++)
        {
            const Vertex &v = (*list)[i];
            float *p = &points[i * 3];
            p[0] = v.z;
            p[1] = v.x;
            p[2] = v.y;
        }
        sendList(type);
    }
    frameId++;
}

void FrameSender::transmit(PacketHeader &header, uint8_t *packet)
{
    header.crc = 0;
    std::memcpy(packet, &header, sizeof(header));
    header.crc = crc32(packet, sizeof(header) + header.payloadSize);
    std::memcpy(packet, &header, sizeof(header));
    sendto(sock, packet, sizeof(header) + header.payloadSize, 0, reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));
}

void FrameSender::sendList(PacketType type)
{
    const int n = pointCount;
    PacketHeader header{};
    header.type = type;
    header.frameId = frameId;
    header.chunkCount = n ? (n + CHUNK_POINTS - 1) / CHUNK_POINTS : 1;

    float scale[3];
    for (int a = 0; a < 3; a++)
    {
        header.min[a] = n ? std::numeric_limits<float>::max() : 0;
        header.max[a] = n ? std::numeric_limits<float>::lowest() : 0;
        for (int i = 0; i < n; i++)
        {
            header.min[a] = std::min(header.min[a], points[i * 3 + a]);
            header.max[a] = std::max(header.max[a], points[i * 3 + a]);
        }
        const float range = header.max[a] - header.min[a];
        scale[a] = range > 0 ? 65534.0f / range : 0;
    }

    int16_t raw[CHUNK_POINTS * 3];
    constexpr int bound = LZ4_COMPRESSBOUND(sizeof(raw));
    uint8_t packet[sizeof(PacketHeader) + bound];

    int totalSize = 0;
    for (int c = 0; c < header.chunkCount; c++)
    {
        const int first = c * CHUNK_POINTS;
        const int count = std::min(CHUNK_POINTS, n - first);
        for (int i = 0; i < count * 3; i++)
        {
            const int a = i % 3;
            raw[i] = static_cast<int16_t>(std::lround((points[first * 3 + i] - header.min[a]) * scale[a]) - 32767);
        }
        const int size = LZ4_compress_default(reinterpret_cast<const char *>(raw),
                                              reinterpret_cast<char *>(packet + sizeof(PacketHeader)),
                                              count * 3 * static_cast<int>(sizeof(int16_t)), bound);
        totalSize += size;
        header.chunkIndex = c;
        header.pointCount = count;
        header.payloadSize = size;
        transmit(header, packet);
    }
    #ifdef FRAME_SENDER_DEBUG
    std::cout << "Total bytes of payload sent: " << totalSize << std::endl;
    #endif
}
