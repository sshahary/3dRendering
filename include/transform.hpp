#pragma once
#include "tinymath.hpp"

class Transform {
public:
    Transform();
    void setTranslation(const tmx::vec3& t);
    void setRotation(float deg, const tmx::vec3& axis);
    void setScale(const tmx::vec3& s);
    tmx::mat4 matrix() const;
private:
    tmx::vec3 T_; float angle_; tmx::vec3 axis_; tmx::vec3 S_;
    mutable tmx::mat4 M_; mutable bool dirty_;
    void rebuild() const;
};
