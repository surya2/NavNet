#include "capture/extract_capture.h"

static Matrices matrices;
static float heights[DEPTH_PIXELS], coordX[DEPTH_PIXELS], coordY[DEPTH_PIXELS], gradients[DEPTH_PIXELS];
static uint8_t valid[DEPTH_PIXELS];

int main()
{
    matrices.heights = heights;
    matrices.coordX = coordX;
    matrices.coordY = coordY;
    matrices.valid = valid;
    matrices.gradients = gradients;
    std::vector<Vertex> vertices;
    if (!runMatrixCollector(matrices, vertices, 4))
        return 1;
    save_to_ply(vertices, "vertices.ply");
    std::cout << "Saved " << vertices.size() << " points to vertices.ply" << std::endl;
    return 0;
}
