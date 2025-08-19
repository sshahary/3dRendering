#include "line_raster.hpp"
#include <algorithm>
LineRaster::LineRaster(int w,int h):w_(w),h_(h),buf_(w*h){} 
void LineRaster::resize(int w,int h){ w_=w; h_=h; buf_.assign(w*h, RGBA{}); }
void LineRaster::clear(const RGBA& c){ std::fill(buf_.begin(), buf_.end(), c); }
void LineRaster::set(int x,int y,const RGBA& c){ if(x<0||y<0||x>=w_||y>=h_) return; buf_[y*w_+x]=c; }
void LineRaster::line(const tmx::ivec2& a,const tmx::ivec2& b,const RGBA& c){
    int x0=a.x, y0=a.y, x1=b.x, y1=b.y;
    bool steep = std::abs(y1-y0)>std::abs(x1-x0);
    if(steep){ std::swap(x0,y0); std::swap(x1,y1); }
    if(x0>x1){ std::swap(x0,x1); std::swap(y0,y1); }
    int dx=x1-x0, dy=std::abs(y1-y0), err=dx/2, ystep=(y0<y1)?1:-1, y=y0;
    for(int x=x0;x<=x1;++x){ if(steep) set(y,x,c); else set(x,y,c); err-=dy; if(err<0){ y+=ystep; err+=dx; } }
}
