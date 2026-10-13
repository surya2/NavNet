#include "capture/extract_capture.h"

int main()
{
    std::vector<Vertex> vertices;
    if (!runMatrixCollector(vertices, 4))
        return 1;
    save_to_ply(vertices, "vertices.ply");
    std::cout << "Saved " << vertices.size() << " points to vertices.ply" << std::endl;
    return 0;
}
