#include "sketch_app.hpp"
#include <algorithm>

SketchApp::SketchApp(const std::vector<tmx::vec3>& positions,
              const std::vector<std::pair<int,int>>& edges,
              int W, int H)
: W_(W),H_(H),center_{0,0,0},dist_(6.0f),eye_{0,0,6},
  M_(tmx::mat4::identity()),V_(tmx::mat4::identity()),P_(tmx::simplePerspective()),
  pipe_(M_,V_,P_),vp_(W,H),rast_(W,H),pos_(positions),edges_(edges)
{ computeCenter(); }

void SketchApp::computeCenter(){
    if(pos_.empty()){ center_={0,0,0}; return; }
    tmx::vec3 mn=pos_[0], mx=pos_[0];
    for(const auto& v: pos_){
        mn.x=std::min(mn.x,v.x); mn.y=std::min(mn.y,v.y); mn.z=std::min(mn.z,v.z);
        mx.x=std::max(mx.x,v.x); mx.y=std::max(mx.y,v.y); mx.z=std::max(mx.z,v.z);
    }
    center_={(mn.x+mx.x)/2.f,(mn.y+mx.y)/2.f,(mn.z+mx.z)/2.f};
}

const std::vector<uint32_t>& SketchApp::render(){
    V_ = tmx::lookAt(eye_, tmx::vec3{0,0,0}, tmx::vec3{0,1,0});
    P_ = (proj_.type()==Projection::Type::Perspective)? tmx::simplePerspective() : proj_.matrix();
    pipe_.setModel(M_); pipe_.setView(V_); pipe_.setProj(P_);
    auto ndc = pipe_.transform(pos_);
    auto scr = vp_.toScreen(ndc);
    rast_.resize(W_,H_); rast_.clear(RGBA{255,255,255,255});
    RGBA black{0,0,0,255};
    for(const auto& e: edges_){
        if(e.first<0||e.second<0||e.first>=(int)scr.size()||e.second>=(int)scr.size()) continue;
        rast_.line({scr[e.first].x,scr[e.first].y},{scr[e.second].x,scr[e.second].y},black);
    }
    const auto& buf = rast_.pixels();
    out_.resize(buf.size());
    for(size_t i=0;i<buf.size();++i){ const auto& c=buf[i]; out_[i]=(uint32_t(c.a)<<24)|(uint32_t(c.r)<<16)|(uint32_t(c.g)<<8)|uint32_t(c.b); }
    return out_;
}

void SketchApp::usePerspective(bool on){ if(on) proj_.setPerspective(60.f,float(W_)/float(H_),0.1f,100.f); else proj_.setOrtho(-1,1,-1,1,-1,1); }
void SketchApp::orbit(float /*dx*/, float /*dy*/){}
void SketchApp::dolly(float factor){ dist_*=factor; }
void SketchApp::resize(int W,int H){ W_=W; H_=H; vp_.resize(W,H); }
