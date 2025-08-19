#include "projection.hpp"
Projection::Projection(){ M_=tmx::simplePerspective(); T_=Type::Perspective; }
void Projection::setPerspective(float /*fov*/, float /*aspect*/, float /*n*/, float /*f*/){ M_=tmx::simplePerspective(); T_=Type::Perspective; }
void Projection::setOrtho(float l,float r,float b,float t,float n,float f){
    tmx::mat4 m{};
    m.m[0]=2.f/(r-l); m.m[5]=2.f/(t-b); m.m[10]=2.f/(f-n);
    m.m[12]=-(r+l)/(r-l); m.m[13]=-(t+b)/(t-b); m.m[14]=-(f+n)/(f-n); m.m[15]=1.f;
    M_=m; T_=Type::Ortho;
}
tmx::mat4 Projection::matrix() const { return M_; }
Projection::Type Projection::type() const { return T_; }
