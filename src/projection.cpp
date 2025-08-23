#include "projection.hpp"

Projection::Projection() : M_(tmx::mat4::identity()), T_(Type::Perspective) {}

void Projection::setPerspective(float fov_deg, float aspect, float n, float f) {
  M_ = tmx::perspective(fov_deg, aspect, n, f);
  T_ = Type::Perspective;
}
void Projection::setOrtho(float l, float r, float b, float t, float n,
                          float f) {
  tmx::mat4 m{};
  m.m[0] = 2.0f / (r - l);
  m.m[5] = 2.0f / (t - b);
  m.m[10] = -2.0f / (f - n);
  m.m[12] = -(r + l) / (r - l);
  m.m[13] = -(t + b) / (t - b);
  m.m[14] = -(f + n) / (f - n);
  m.m[15] = 1.0f;
  M_ = m;
  T_ = Type::Ortho;
}
tmx::mat4 Projection::matrix() const { return M_; }
Projection::Type Projection::type() const { return T_; }
