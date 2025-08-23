#include "pipeline3d.hpp"
#include "parallel.hpp"
#include <cmath>

Pipeline3D::Pipeline3D(const tmx::mat4 &M, const tmx::mat4 &V,
                       const tmx::mat4 &P)
    : M_(M), V_(V), P_(P) {}
void Pipeline3D::setModel(const tmx::mat4 &m) { M_ = m; }
void Pipeline3D::setView(const tmx::mat4 &v) { V_ = v; }
void Pipeline3D::setProj(const tmx::mat4 &p) { P_ = p; }

std::vector<tmx::vec4>
Pipeline3D::transform(const std::vector<tmx::vec3> &vs) const {
  std::vector<tmx::vec4> out;
  out.reserve(vs.size());
  tmx::mat4 MVP = P_ * (V_ * M_);
  for (const auto &v : vs) {
    tmx::vec4 p{v.x, v.y, v.z, 1.0f};
    out.push_back(MVP * p);
  }
  return out;
}

void Pipeline3D::transformInto(const std::vector<tmx::vec3> &vs,
                               std::vector<tmx::vec4> &out) const {
  out.resize(vs.size());
  tmx::mat4 MVP = P_ * (V_ * M_);
  for (size_t i = 0; i < vs.size(); ++i) {
    const auto &v = vs[i];
    tmx::vec4 p{v.x, v.y, v.z, 1.0f};
    tmx::vec4 q = MVP * p;
    out[i] = q;
  }
}

void Pipeline3D::transformIntoParallel(const std::vector<tmx::vec3> &vs,
                                       std::vector<tmx::vec4> &out,
                                       unsigned threads) const {
  out.resize(vs.size());
  tmx::mat4 MVP = P_ * (V_ * M_);
  par::parallel_for(
      0, vs.size(),
      [&](std::size_t i) {
        const auto &v = vs[i];
        tmx::vec4 p{v.x, v.y, v.z, 1.0f};
        tmx::vec4 q = MVP * p;
        out[i] = q;
      },
      threads);
}
