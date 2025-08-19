#include "camera.hpp"
Camera::Camera():eye_{0,0,5},target_{0,0,0},up_{0,1,0},dirty_(true){} 
void Camera::setLookAt(const tmx::vec3& e,const tmx::vec3& t,const tmx::vec3& u){ eye_=e; target_=t; up_=u; dirty_=true; }
void Camera::setEye(const tmx::vec3& e){ eye_=e; dirty_=true; } void Camera::setTarget(const tmx::vec3& t){ target_=t; dirty_=true; } void Camera::setUp(const tmx::vec3& u){ up_=u; dirty_=true; }
tmx::vec3 Camera::eye() const { return eye_; } tmx::vec3 Camera::target() const { return target_; } tmx::vec3 Camera::up() const { return up_; }
void Camera::rebuild() const { view_ = tmx::lookAt(eye_, target_, up_); dirty_ = false; }
tmx::mat4 Camera::view() const { if(dirty_) rebuild(); return view_; }
