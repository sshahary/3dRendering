#include "transform.hpp"
#include <algorithm>
#include <cmath>

static inline tmx::mat4 makeScale(const tmx::vec3 &s) {
  tmx::mat4 m = tmx::mat4::identity();
  m.m[0] = s.x;
  m.m[5] = s.y;
  m.m[10] = s.z;
  return m;
}

static inline tmx::mat4 makeRotationAxisAngle(float deg, tmx::vec3 axis) {
  float len = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
  if (len == 0.f)
    return tmx::mat4::identity();
  axis.x /= len;
  axis.y /= len;
  axis.z /= len;

  const float r = deg * 3.1415926535f / 180.f;
  const float c = std::cos(r), s = std::sin(r), t = 1.f - c;
  const float x = axis.x, y = axis.y, z = axis.z;

  tmx::mat4 m = tmx::mat4::identity();
  m.m[0] = t * x * x + c;
  m.m[4] = t * x * y + s * z;
  m.m[8] = t * x * z - s * y;
  m.m[1] = t * x * y - s * z;
  m.m[5] = t * y * y + c;
  m.m[9] = t * y * z + s * x;
  m.m[2] = t * x * z + s * y;
  m.m[6] = t * y * z - s * x;
  m.m[10] = t * z * z + c;
  return m;
}

Transform::Transform() = default;

void Transform::setTranslation(const tmx::vec3 &t) {
  T_ = t;
  dirty_ = true;
}
void Transform::setRotation(float degrees, const tmx::vec3 &axis) {
  angleDeg_ = degrees;
  axis_ = axis;
  dirty_ = true;
}
void Transform::setScale(const tmx::vec3 &s) {
  S_ = s;
  dirty_ = true;
}
void Transform::setPivot(const tmx::vec3 &p) {
  pivot_ = p;
  dirty_ = true;
}
tmx::mat4 Transform::matrix() const {
  if (dirty_)
    rebuild();
  return M_;
}

void Transform::rebuild() const {
  const tmx::mat4 Tp = tmx::mat4::translation(pivot_.x, pivot_.y, pivot_.z);
  const tmx::mat4 Tpinv =
      tmx::mat4::translation(-pivot_.x, -pivot_.y, -pivot_.z);
  const tmx::mat4 T = tmx::mat4::translation(T_.x, T_.y, T_.z);
  const tmx::mat4 R = makeRotationAxisAngle(angleDeg_, axis_);
  const tmx::mat4 S = makeScale(S_);
  M_ = T * Tp * R * S * Tpinv;
  dirty_ = false;
}
