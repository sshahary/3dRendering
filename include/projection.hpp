#pragma once
#include "tinymath.hpp"

class Projection {
public:
    enum class Type { Perspective, Ortho };
    Projection();
    void setPerspective(float fov, float aspect, float n, float f);
    void setOrtho(float l,float r,float b,float t,float n,float f);
    tmx::mat4 matrix() const;
    Type type() const;
private:
    tmx::mat4 M_; Type T_;
};
