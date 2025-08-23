#pragma once
#include "tinymath.hpp"

class Camera {
public:
  Camera();
  void setLookAt(const tmx::vec3 &eye, const tmx::vec3 &target,
                 const tmx::vec3 &up);
  void setEye(const tmx::vec3 &eye);
  void setTarget(const tmx::vec3 &t);
  void setUp(const tmx::vec3 &u);
  tmx::vec3 eye() const;
  tmx::vec3 target() const;
  tmx::vec3 up() const;
  tmx::mat4 view() const;

private:
  tmx::vec3 eye_, target_, up_;
  mutable tmx::mat4 view_;
  mutable bool dirty_;
  void rebuild() const;
};
