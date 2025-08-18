#pragma once
#include <string>
#include <vector>
#include <utility>

struct PolyFace { std::vector<int> indices; };

class MeshLoader {
public:
    std::vector<int> positions;
    std::vector<PolyFace> faces;
    std::vector<std::pair<int,int>> edges;

    bool loadOBJ(const std::string& path);

private:
    void extractEdges();
};
