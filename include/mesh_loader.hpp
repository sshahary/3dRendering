#pragma once
#include "tinymath.hpp"
#include <string>
#include <utility>
#include <vector>

struct PolyFace {
  std::vector<int> indices;
};

class MeshLoader {
public:
  std::vector<tmx::vec3> positions;
  std::vector<PolyFace> faces;
  std::vector<std::pair<int, int>> edges;

  bool loadOBJ(const std::string &path);

private:
  void extractEdges();
};
