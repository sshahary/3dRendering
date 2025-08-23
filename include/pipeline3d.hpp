#pragma once
#include "tinymath.hpp"
#include <vector>

class Pipeline3D {
public:
  Pipeline3D(const tmx::mat4 &M, const tmx::mat4 &V, const tmx::mat4 &P);
  void setModel(const tmx::mat4 &m);
  void setView(const tmx::mat4 &v);
  void setProj(const tmx::mat4 &p);
  std::vector<tmx::vec4> transform(const std::vector<tmx::vec3> &vs) const;
  void transformInto(const std::vector<tmx::vec3> &vs,
                     std::vector<tmx::vec4> &out) const;
  void transformIntoParallel(const std::vector<tmx::vec3> &vs,
                             std::vector<tmx::vec4> &out,
                             unsigned threads) const;

private:
  tmx::mat4 M_, V_, P_;
};
