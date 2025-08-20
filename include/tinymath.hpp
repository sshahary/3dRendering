
#pragma once
#include <cmath>
#include <cstdint>

namespace tmx {

struct ivec2 { int x{}, y{}; };

struct vec3 {
    float x{}, y{}, z{};
    vec3() = default;
    vec3(float X,float Y,float Z):x(X),y(Y),z(Z){}
};

struct vec4 {
    float x{}, y{}, z{}, w{};
    vec4() = default;
    vec4(float X,float Y,float Z,float W):x(X),y(Y),z(Z),w(W){}
};

struct mat4 {
    float m[16]{}; // column-major
    static mat4 identity() {
        mat4 r{}; r.m[0]=r.m[5]=r.m[10]=r.m[15]=1.0f; return r;
    }
    static mat4 translation(float tx,float ty,float tz) {
        mat4 r = identity(); r.m[12]=tx; r.m[13]=ty; r.m[14]=tz; return r;
    }
    static mat4 scale(float sx,float sy,float sz) {
        mat4 r{}; r.m[0]=sx; r.m[5]=sy; r.m[10]=sz; r.m[15]=1.0f; return r;
    }
    static mat4 rotationAxis(float a, const vec3& axis) {
        float c=std::cos(a), s=std::sin(a), ic=1.0f-c;
        float x=axis.x, y=axis.y, z=axis.z;
        mat4 r = identity();
        r.m[0]=c+x*x*ic;   r.m[4]=x*y*ic - z*s; r.m[8]=x*z*ic + y*s;
        r.m[1]=y*x*ic + z*s; r.m[5]=c + y*y*ic; r.m[9]=y*z*ic - x*s;
        r.m[2]=z*x*ic - y*s; r.m[6]=z*y*ic + x*s; r.m[10]=c + z*z*ic;
        return r;
    }
};

inline vec3 operator+(const vec3&a,const vec3&b){ return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline vec3 operator-(const vec3&a,const vec3&b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline vec3 operator*(const vec3&a,float s){ return {a.x*s,a.y*s,a.z*s}; }
inline float dot(const vec3&a,const vec3&b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
inline float length(const vec3&v){ return std::sqrt(dot(v,v)); }
inline vec3 normalize(const vec3&v){ float L=length(v); return (L>1e-6f)? vec3{v.x/L,v.y/L,v.z/L} : vec3{}; }

inline mat4 operator*(const mat4&a,const mat4&b){
    mat4 r{};
    for(int c=0;c<4;++c)for(int r_i=0;r_i<4;++r_i){
        r.m[c*4+r_i] =
            a.m[0*4+r_i]*b.m[c*4+0] +
            a.m[1*4+r_i]*b.m[c*4+1] +
            a.m[2*4+r_i]*b.m[c*4+2] +
            a.m[3*4+r_i]*b.m[c*4+3];
    }
    return r;
}

inline vec4 operator*(const mat4&a,const vec4&v){
    vec4 r;
    r.x=a.m[0]*v.x + a.m[4]*v.y + a.m[8]*v.z + a.m[12]*v.w;
    r.y=a.m[1]*v.x + a.m[5]*v.y + a.m[9]*v.z + a.m[13]*v.w;
    r.z=a.m[2]*v.x + a.m[6]*v.y + a.m[10]*v.z + a.m[14]*v.w;
    r.w=a.m[3]*v.x + a.m[7]*v.y + a.m[11]*v.z + a.m[15]*v.w;
    return r;
}

inline mat4 lookAt(const vec3& eye, const vec3& target, const vec3& up) {
    vec3 cz = normalize(vec3{eye.x-target.x,eye.y-target.y,eye.z-target.z});
    vec3 cx = normalize(vec3{ up.y*cz.z - up.z*cz.y, up.z*cz.x - up.x*cz.z, up.x*cz.y - up.y*cz.x });
    vec3 cy = vec3{ cz.y*cx.z - cz.z*cx.y, cz.z*cx.x - cz.x*cx.z, cz.x*cx.y - cz.y*cx.x };
    mat4 r = mat4::identity();
    r.m[0]=cx.x; r.m[4]=cx.y; r.m[8]=cx.z;
    r.m[1]=cy.x; r.m[5]=cy.y; r.m[9]=cy.z;
    r.m[2]=cz.x; r.m[6]=cz.y; r.m[10]=cz.z;
    r.m[12]=-(cx.x*eye.x + cx.y*eye.y + cx.z*eye.z);
    r.m[13]=-(cy.x*eye.x + cy.y*eye.y + cy.z*eye.z);
    r.m[14]=-(cz.x*eye.x + cz.y*eye.y + cz.z*eye.z);
    return r;
}
// After simplePerspective(), add:
inline mat4 perspective(float fovy_deg, float aspect, float n, float f){
    const float s = std::tan(0.5f * fovy_deg * 3.1415926535f / 180.f);
    mat4 m{};                        // OpenGL-style clip space, z in [-1,1]
    m.m[0]  = 1.f / (aspect * s);
    m.m[5]  = 1.f / s;
    m.m[10] = (f + n) / (n - f);
    m.m[11] = -1.f;
    m.m[14] = (2.f * f * n) / (n - f);
    return m;
}

}
