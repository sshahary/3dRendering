#include "transform.hpp"
Transform::Transform():T_{0,0,0},angle_(0),axis_{0,1,0},S_{1,1,1},dirty_(true){} 
void Transform::setTranslation(const tmx::vec3& t){ T_=t; dirty_=true; }
void Transform::setRotation(float deg,const tmx::vec3& ax){ angle_=deg; axis_=ax; dirty_=true; }
void Transform::setScale(const tmx::vec3& s){ S_=s; dirty_=true; }
void Transform::rebuild() const{
    float a = angle_*3.1415926535f/180.f;
    M_ = tmx::mat4::translation(T_.x,T_.y,T_.z) * tmx::mat4::rotationAxis(a, axis_) * tmx::mat4::scale(S_.x,S_.y,S_.z);
    dirty_=false;
}
tmx::mat4 Transform::matrix() const { if(dirty_) rebuild(); return M_; }
