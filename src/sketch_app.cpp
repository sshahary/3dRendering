
#include "sketch_app.hpp"
#include <algorithm>
#include <cmath>

SketchApp::SketchApp(const std::vector<int>& positions,
    const std::vector<std::pair<int,int>>& edges,
    int W, int H) : W_(W), H_(H),
    pos_(positions),     // 4) out_ is default-inited later; then pos_
    edges_(edges)       // 5) edges_
    {}

const std::vector<uint32_t>& SketchApp::render(){
    auto d2r = [](float d){ return d * 3.1415926535f/180.0f; };
    }

    // Trivial reject: skip edges that are outside the same clip plane

void SketchApp::dolly(float factor){
    dist_ *= factor;
    if(dist_ < 0.2f) dist_ = 0.2f;
    if(dist_ > 2000.f) dist_ = 2000.f;
}
void SketchApp::resize(int W,int H){ W_=W; H_=H; resize(W,H); }
