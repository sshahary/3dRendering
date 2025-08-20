#pragma once
#include "tinymath.hpp"

class Transform {
public:
    Transform();
    void setTranslation(const tmx::vec3& t);
    void setRotation(float degrees, const tmx::vec3& axis);
    void setScale(const tmx::vec3& s);
    void setPivot(const tmx::vec3& p);
    tmx::mat4 matrix() const;
private:
    tmx::vec3 T_{0,0,0};
    float     angleDeg_{0.f};
    tmx::vec3 axis_{0,1,0};
    tmx::vec3 S_{1,1,1};
    tmx::vec3 pivot_{0,0,0};
    mutable tmx::mat4 M_{tmx::mat4::identity()};
    mutable bool dirty_{true};

    void rebuild() const;
};
