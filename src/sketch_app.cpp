#include "sketch_app.hpp"
#include <algorithm>
#include <cmath>
#include "parallel.hpp"

// ---------- helpers ----------
tmx::vec3 SketchApp::computeCenter(const std::vector<tmx::vec3>& v){
    if (v.empty()) return {0,0,0};
    tmx::vec3 mn=v[0], mx=v[0];
    for (auto& p: v){
        mn.x=std::min(mn.x,p.x); mn.y=std::min(mn.y,p.y); mn.z=std::min(mn.z,p.z);
        mx.x=std::max(mx.x,p.x); mx.y=std::max(mx.y,p.y); mx.z=std::max(mx.z,p.z);
    }
    return {(mn.x+mx.x)*0.5f,(mn.y+mx.y)*0.5f,(mn.z+mx.z)*0.5f};
}

// ---------- ctor ----------
SketchApp::SketchApp(const std::vector<tmx::vec3>& positions,
                     const std::vector<std::pair<int,int>>& edges,
                     int W, int H)
: W_(W), H_(H),
  center_(computeCenter(positions)),
  // translate model to origin so orbit is about the true center
  M_(tmx::mat4::translation(-center_.x, -center_.y, -center_.z)),
  V_(tmx::mat4::identity()),
  P_(tmx::simplePerspective()),
  pipe_(M_,V_,P_), vp_(W,H), rast_(W,H),
  pos_(positions), edges_(edges)
{
    // nothing else — keep it simple & stable
}

// ---------- controls ----------
void SketchApp::orbit(float dx, float dy){
    yaw_deg_   += dx;                     // unbounded yaw
    pitch_deg_ += dy;
    if (pitch_deg_ >  89.f) pitch_deg_ =  89.f;
    if (pitch_deg_ < -89.f) pitch_deg_ = -89.f;
}
void SketchApp::dolly(float factor){
    dist_ *= factor;
    if (dist_ < 0.2f)   dist_ = 0.2f;
    if (dist_ > 2000.f) dist_ = 2000.f;
}
void SketchApp::resize(int W,int H){ W_=W; H_=H; vp_.resize(W,H); }
void SketchApp::usePerspective(bool on){
    if(on) P_ = tmx::simplePerspective();
    else   P_ = tmx::mat4::identity();
}

// ---------- render ----------
const std::vector<uint32_t>& SketchApp::render(){
    // keep yaw bounded (optional)
    if (yaw_deg_ >  360.f) yaw_deg_ = std::fmod(yaw_deg_, 360.f);
    if (yaw_deg_ < -360.f) yaw_deg_ = std::fmod(yaw_deg_, 360.f);

    auto d2r = [](float d){ return d * 3.1415926535f/180.0f; };
    float yaw = d2r(yaw_deg_), pitch = d2r(pitch_deg_);

    // yaw=0 → +Z (front), pitch=0 level
    tmx::vec3 eye{
        std::sin(yaw) * std::cos(pitch) * dist_,
        std::sin(pitch) * dist_,
        std::cos(yaw) * std::cos(pitch) * dist_
    };

    V_ = tmx::lookAt(eye, tmx::vec3{0,0,0}, tmx::vec3{0,1,0});
    pipe_.setModel(M_); pipe_.setView(V_); pipe_.setProj(P_);

    // 1) parallel vertex transform → NDC (x,y ∈ [-1,1], z ∈ [0,1])
    pipe_.transformIntoParallel(pos_, ndc_, threads_ ? threads_ : par::hw_threads());

    // 2) parallel map to screen + outcodes
    scr_.resize(ndc_.size());
    codes_.resize(ndc_.size());
    par::parallel_for(0, ndc_.size(), [&](std::size_t i){
        const auto& p = ndc_[i];
        uint8_t c=0;
        if(p.x<-1.f) c|=1; if(p.x>1.f) c|=2;
        if(p.y<-1.f) c|=4; if(p.y>1.f) c|=8;
        if(p.z< 0.f) c|=16; if(p.z>1.f) c|=32;
        codes_[i]=c;

        int x = int((p.x*0.5f + 0.5f) * float(W_));
        int y = int((1.f - (p.y*0.5f + 0.5f)) * float(H_));
        scr_[i] = {x,y};
    }, threads_ ? threads_ : par::hw_threads());

    // 3) clear every frame (so zooming inside never leaves “old” edges)
    rast_.resize(W_,H_);
    rast_.clear({255,255,255,255});
    const RGBA black{0,0,0,255};

    // optional macro-grid to reduce overdraw on huge meshes
    if (grid_px_ > 0){
        gridW_ = std::max(1, W_ / grid_px_);
        gridH_ = std::max(1, H_ / grid_px_);
        gridOcc_.assign(gridW_ * gridH_, 0);
    }
    auto okMacro = [&](int x0,int y0,int x1,int y1)->bool{
        if (grid_px_<=0) return true;
        int c0x = std::clamp(x0 / grid_px_, 0, gridW_-1);
        int c0y = std::clamp(y0 / grid_px_, 0, gridH_-1);
        int c1x = std::clamp(x1 / grid_px_, 0, gridW_-1);
        int c1y = std::clamp(y1 / grid_px_, 0, gridH_-1);
        int i0 = c0y*gridW_ + c0x, i1 = c1y*gridW_ + c1x;
        if (gridOcc_[i0] && gridOcc_[i1]) return false;
        gridOcc_[i0] = gridOcc_[i1] = 1;
        return true;
    };

    // 4) draw edges (stride + LOD + trivial reject), no accumulation
    std::size_t drawn = 0;
    const float minLen2 = min_len_px_ * min_len_px_;
    for (size_t ei=0; ei<edges_.size(); ei += edge_stride_){
        if (drawn >= max_edges_) break;
        const auto& e = edges_[ei];

        if (e.first  < 0 || e.second < 0) continue;
        if (e.first  >= (int)scr_.size() || e.second >= (int)scr_.size()) continue;

        if ((codes_[e.first] & codes_[e.second]) != 0) continue; // outside

        const auto a = scr_[e.first];
        const auto b = scr_[e.second];

        float dx = float(a.x - b.x), dy = float(a.y - b.y);
        if ((dx*dx + dy*dy) < minLen2) continue;
        if (!okMacro(a.x,a.y,b.x,b.y)) continue;

        rast_.line(a, b, black);
        ++drawn;
    }

    const auto& buf = rast_.pixels();
    out_.resize(buf.size());
    for (size_t i=0;i<buf.size();++i){
        const auto& c = buf[i];
        out_[i] = (uint32_t(c.a)<<24) | (uint32_t(c.r)<<16) | (uint32_t(c.g)<<8) | uint32_t(c.b);
    }
    return out_;
}
