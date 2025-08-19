#include "viewport.hpp"
Viewport::Viewport(int w,int h):w_(w),h_(h){} 
void Viewport::resize(int w,int h){ w_=w; h_=h; }
std::vector<tmx::ivec2> Viewport::toScreen(const std::vector<tmx::vec4>& ndc) const {
    std::vector<tmx::ivec2> out; out.reserve(ndc.size());
    for(const auto& p: ndc){
        int x = int((p.x*0.5f+0.5f)*float(w_));
        int y = int((1.0f-(p.y*0.5f+0.5f))*float(h_));
        out.push_back({x,y});
    }
    return out;
}
