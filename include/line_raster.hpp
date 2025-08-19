#pragma once
#include <vector>
#include "tinymath.hpp"

struct RGBA { unsigned char r,g,b,a; RGBA(unsigned char R=0,unsigned char G=0,unsigned char B=0,unsigned char A=255):r(R),g(G),b(B),a(A){} };

class LineRaster {
public:
    LineRaster(int w,int h);
    void resize(int w,int h);
    void clear(const RGBA& c);
    void line(const tmx::ivec2& a, const tmx::ivec2& b, const RGBA& c);
    const std::vector<RGBA>& pixels() const { return buf_; }
    int width() const { return w_; } int height() const { return h_; }
private:
    int w_, h_; std::vector<RGBA> buf_;
    void set(int x,int y,const RGBA& c);
};
