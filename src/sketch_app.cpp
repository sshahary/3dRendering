#include "sketch_app.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include "parallel.hpp"

// Simple Cohen–Sutherland clip for integer rectangle
static inline bool clipLineRect(int x0,int y0,int x1,int y1, int rx0,int ry0,int rx1,int ry1,
                                int& ox0,int& oy0,int& ox1,int& oy1)
{
    auto outcode = [&](int x,int y){
        int c=0;
        if(x<rx0) c|=1; else if(x>rx1) c|=2;
        if(y<ry0) c|=4; else if(y>ry1) c|=8;
        return c;
    };
    int c0=outcode(x0,y0), c1=outcode(x1,y1);
    while(true){
        if((c0|c1)==0){ ox0=x0; oy0=y0; ox1=x1; oy1=y1; return true; }
        if((c0&c1)!=0) return false;
        int c = c0? c0:c1;
        int x=0,y=0;
        if(c&8){ x = x0 + (x1-x0)*(ry1 - y0)/(y1 - y0); y = ry1; }
        else if(c&4){ x = x0 + (x1-x0)*(ry0 - y0)/(y1 - y0); y = ry0; }
        else if(c&2){ y = y0 + (y1-y0)*(rx1 - x0)/(x1 - x0); x = rx1; }
        else        { y = y0 + (y1-y0)*(rx0 - x0)/(x1 - x0); x = rx0; }
        if(c==c0){ x0=x; y0=y; c0=outcode(x0,y0); }
        else     { x1=x; y1=y; c1=outcode(x1,y1); }
    }
}

static inline tmx::vec3 computeCenter(const std::vector<tmx::vec3>& v){
    if(v.empty()) return {0,0,0};
    tmx::vec3 mn = v[0], mx = v[0];
    for(const auto& p: v){
        mn.x = std::min(mn.x, p.x); mn.y = std::min(mn.y, p.y); mn.z = std::min(mn.z, p.z);
        mx.x = std::max(mx.x, p.x); mx.y = std::max(mx.y, p.y); mx.z = std::max(mx.z, p.z);
    }
    return tmx::vec3{ (mn.x+mx.x)*0.5f, (mn.y+mx.y)*0.5f, (mn.z+mx.z)*0.5f };
}

SketchApp::SketchApp(const std::vector<tmx::vec3>& positions,
                     const std::vector<std::pair<int,int>>& edges,
                     int W, int H)
: W_(W), H_(H),
  center_(computeCenter(positions)),
  M_(tmx::mat4::translation(-center_.x, -center_.y, -center_.z)), // center model at origin
  V_(tmx::mat4::identity()),
  P_(tmx::simplePerspective()),
  pipe_(M_,V_,P_), vp_(W,H), rast_(W,H),
  pos_(positions), edges_(edges)
{
}

void SketchApp::orbit(float dx, float dy){
    yaw_deg_   += dx;              // unbounded
    pitch_deg_ += dy;
    if(pitch_deg_>89.f)  pitch_deg_=89.f;
    if(pitch_deg_<-89.f) pitch_deg_=-89.f;
}
void SketchApp::dolly(float factor){
    dist_ *= factor;
    if(dist_<0.2f) dist_=0.2f;
    if(dist_>2000.f) dist_=2000.f;
}
void SketchApp::resize(int W,int H){ W_=W; H_=H; vp_.resize(W,H); }

const std::vector<uint32_t>& SketchApp::render(){
    // wrap yaw to keep numbers stable
    if (yaw_deg_ >  360.f) yaw_deg_ = std::fmod(yaw_deg_, 360.f);
    if (yaw_deg_ < -360.f) yaw_deg_ = std::fmod(yaw_deg_, 360.f);

    // camera (orbit around origin, since model is centered by M_)
    auto d2r = [](float d){ return d * 3.1415926535f/180.0f; };
    float cy = d2r(yaw_deg_), cp = d2r(pitch_deg_);
    tmx::vec3 eye{ std::cos(cp)*std::cos(cy)*dist_,
                   std::sin(cp)*dist_,
                   std::cos(cp)*std::sin(cy)*dist_ };

    V_ = tmx::lookAt(eye, tmx::vec3{0,0,0}, tmx::vec3{0,1,0});
    pipe_.setView(V_), pipe_.setModel(M_), pipe_.setProj(P_);

    // 1) Parallel vertex transform → clip-space (actually NDC after perspective divide in your Pipeline3D)
    pipe_.transformIntoParallel(pos_, clip_, threads_ ? threads_ : par::hw_threads());

    // 2) Parallel screen mapping + outcodes
    scr_.resize(clip_.size()); codes_.resize(clip_.size());
    par::parallel_for(0, clip_.size(), [&](std::size_t i){
        const auto& p = clip_[i]; // assume already divided by w in Pipeline3D
        uint8_t c=0;
        if(p.x < -1.f) c|=1; if(p.x > 1.f) c|=2;
        if(p.y < -1.f) c|=4; if(p.y > 1.f) c|=8;
        if(p.z <  0.f) c|=16; if(p.z > 1.f) c|=32;
        codes_[i]=c;
        int x = int((p.x*0.5f+0.5f)*float(W_));
        int y = int((1.0f-(p.y*0.5f+0.5f))*float(H_));
        scr_[i] = {x,y};
    }, threads_ ? threads_ : par::hw_threads());

    // 3) Build tiles and bin edges (race-free tiling)
    constexpr int TILE=64;
    int gridW = (W_ + TILE - 1)/TILE;
    int gridH = (H_ + TILE - 1)/TILE;

    struct Tile { int x0,y0,x1,y1; std::vector<int> e; };
    std::vector<Tile> tiles(gridW*gridH);
    for(int ty=0; ty<gridH; ++ty)
    for(int tx=0; tx<gridW; ++tx){
        auto& t = tiles[ty*gridW+tx];
        t.x0 = tx*TILE; t.y0 = ty*TILE;
        t.x1 = std::min(t.x0 + TILE - 1, W_-1);
        t.y1 = std::min(t.y0 + TILE - 1, H_-1);
    }

    // Quick LOD: skip very short edges in screen space (<2px)
    const int minPix2 = 2*2;
    for(size_t i=0;i<edges_.size();++i){
        const auto& e = edges_[i];
        if((codes_[e.first] & codes_[e.second])!=0) continue; // trivial reject
        const auto a=scr_[e.first], b=scr_[e.second];
        int dx=a.x-b.x, dy=a.y-b.y; if(dx*dx+dy*dy < minPix2) continue;

        int minx=std::min(a.x,b.x), miny=std::min(a.y,b.y);
        int maxx=std::max(a.x,b.x), maxy=std::max(a.y,b.y);
        int tx0=std::clamp(minx / TILE, 0, gridW-1);
        int ty0=std::clamp(miny / TILE, 0, gridH-1);
        int tx1=std::clamp(maxx / TILE, 0, gridW-1);
        int ty1=std::clamp(maxy / TILE, 0, gridH-1);
        for(int ty=ty0; ty<=ty1; ++ty)
            for(int tx=tx0; tx<=tx1; ++tx)
                tiles[ty*gridW+tx].e.push_back((int)i);
    }

    // 4) Clear and draw per-tile in parallel (no races)
    rast_.resize(W_,H_);
    rast_.clear({255,255,255,255});
    auto worker = [&](int t0,int t1){
        for(int ti=t0; ti<t1; ++ti){
            const auto& T = tiles[ti];
            for(int idx : T.e){
                const auto& e = edges_[idx];
                auto a = scr_[e.first];
                auto b = scr_[e.second];
                int x0,y0,x1,y1;
                if(clipLineRect(a.x,a.y,b.x,b.y, T.x0,T.y0,T.x1,T.y1, x0,y0,x1,y1))
                    rast_.line({x0,y0},{x1,y1}, {0,0,0,255});
            }
        }
    };

    int Tn = (int)(threads_? threads_ : par::hw_threads());
    int Ntiles = (int)tiles.size();
    int per = (Ntiles + Tn - 1)/Tn;
    std::vector<std::thread> ts; ts.reserve(Tn);
    for(int t=0;t<Tn;++t){
        int s=t*per, e=std::min(Ntiles, s+per);
        if(s<e) ts.emplace_back(worker,s,e);
    }
    for(auto& th: ts) th.join();

    // 5) pack ARGB
    const auto& buf = rast_.pixels(); out_.resize(buf.size());
    for(size_t i=0;i<buf.size();++i){
        const auto& c = buf[i];
        out_[i] = (uint32_t(c.a)<<24) | (uint32_t(c.r)<<16) | (uint32_t(c.g)<<8) | uint32_t(c.b);
    }
    return out_;
}
