#include "sketch_app.hpp"
#include <algorithm>
#include <cmath>
#include <thread>
#include <cstring>
#include "parallel.hpp"
#include "mesh_loader.hpp" 

// --- small math helpers ---
static inline tmx::vec3 bboxCenter(const std::vector<tmx::vec3>& v){
    if(v.empty()) return {0,0,0};
    tmx::vec3 mn=v[0], mx=v[0];
    for(const auto& p: v){
        mn.x=std::min(mn.x,p.x); mn.y=std::min(mn.y,p.y); mn.z=std::min(mn.z,p.z);
        mx.x=std::max(mx.x,p.x); mx.y=std::max(mx.y,p.y); mx.z=std::max(mx.z,p.z);
    }
    return {(mn.x+mx.x)*0.5f,(mn.y+mx.y)*0.5f,(mn.z+mx.z)*0.5f};
}
static inline tmx::vec3 bboxExtent(const std::vector<tmx::vec3>& v){
    if(v.empty()) return {1,1,1};
    tmx::vec3 mn=v[0], mx=v[0];
    for(const auto& p: v){
        mn.x=std::min(mn.x,p.x); mn.y=std::min(mn.y,p.y); mn.z=std::min(mn.z,p.z);
        mx.x=std::max(mx.x,p.x); mx.y=std::max(mx.y,p.y); mx.z=std::max(mx.z,p.z);
    }
    return {mx.x-mn.x, mx.y-mn.y, mx.z-mn.z};
}
static inline tmx::mat4 rotateX(float r){
    tmx::mat4 m = tmx::mat4::identity();
    float c = std::cos(r), s = std::sin(r);
    // column-major basis (diag at 0,5,10,15)
    m.m[5]  =  c; m.m[6]  =  s;
    m.m[9]  = -s; m.m[10] =  c;
    return m;
}

// ----- ctor -----
SketchApp::SketchApp(const std::vector<tmx::vec3>& positions,
                     const std::vector<std::pair<int,int>>& edges,
                     int W, int H)
: W_(W), H_(H),
  // center model at origin
  M_(tmx::mat4::translation( -bboxCenter(positions).x,
                             -bboxCenter(positions).y,
                             -bboxCenter(positions).z )),
  V_(tmx::mat4::identity()),
  P_(tmx::simplePerspective()),
  pipe_(M_,V_,P_), vp_(W,H), rast_(W,H),
  pos_(positions), edges_(edges)
{
    // auto-upright (Z-up → Y-up) heuristic
    tmx::vec3 ext = bboxExtent(positions);
    if (ext.z > ext.y * 1.2f) {
        M_ = rotateX(-3.1415926535f/2.0f) * M_;
    }
}

// ----- camera controls -----
void SketchApp::orbit(float dx, float dy){
    yaw_deg_   += dx;                // unbounded; we’ll wrap in render()
    pitch_deg_ += dy;
    if(pitch_deg_> 89.f) pitch_deg_= 89.f;
    if(pitch_deg_<-89.f) pitch_deg_=-89.f;
}
void SketchApp::dolly(float factor){
    dist_ *= factor;
    if(dist_<0.2f)   dist_=0.2f;
    if(dist_>2000.f) dist_=2000.f;
}
void SketchApp::resize(int W,int H){
    W_=W; H_=H; vp_.resize(W,H);
    zbuf_.assign((size_t)W_*H_, 1.0f); // keep sized
}

// ----- depth fill for hidden-line -----
static inline float edgeFunc(float x, float y, float x0, float y0, float x1, float y1){
    return (y0 - y1)*x + (x1 - x0)*y + (x0*y1 - x1*y0);
}
static void fillDepthTri(const tmx::ivec2& a, float za,
                         const tmx::ivec2& b, float zb,
                         const tmx::ivec2& c, float zc,
                         int W, int H, std::vector<float>& Z)
{
    int minx = std::max(0, std::min({a.x,b.x,c.x}));
    int maxx = std::min(W-1, std::max({a.x,b.x,c.x}));
    int miny = std::max(0, std::min({a.y,b.y,c.y}));
    int maxy = std::min(H-1, std::max({a.y,b.y,c.y}));
    if(minx>maxx || miny>maxy) return;

    float area = edgeFunc((float)a.x,(float)a.y,(float)b.x,(float)b.y,(float)c.x,(float)c.y);
    if(area==0.f) return;
    float invA = 1.0f/area;

    float A01=a.y-b.y, B01=b.x-a.x, C01=a.x*b.y-b.x*a.y;
    float A12=b.y-c.y, B12=c.x-b.x, C12=b.x*c.y-c.x*b.y;
    float A20=c.y-a.y, B20=a.x-c.x, C20=c.x*a.y-a.x*c.y;

    for(int y=miny; y<=maxy; ++y){
        for(int x=minx; x<=maxx; ++x){
            float w0=(A12*x+B12*y+C12)*invA;
            float w1=(A20*x+B20*y+C20)*invA;
            float w2=(A01*x+B01*y+C01)*invA;
            if(w0<0||w1<0||w2<0) continue;
            float z = w0*za + w1*zb + w2*zc;   // NDC z
            float& zp = Z[(size_t)y*W + x];
            if(z < zp) zp = z;
        }
    }
}
void SketchApp::buildDepth(){
    if(!faces_) return;
    std::fill(zbuf_.begin(), zbuf_.end(), 1.0f);
    // clip_/scr_ already computed for current frame
    for(const auto& f : *faces_){
        if (f.indices.size() < 3) continue;
        int i0 = f.indices[0];
        auto p0 = scr_[i0]; float z0 = clip_[i0].z;
        for (size_t k=1; k+1<f.indices.size(); ++k){
            int i1=f.indices[k], i2=f.indices[k+1];
            auto p1=scr_[i1]; float z1=clip_[i1].z;
            auto p2=scr_[i2]; float z2=clip_[i2].z;
            fillDepthTri(p0,z0,p1,z1,p2,z2, W_,H_, zbuf_);
        }
    }
}

// ----- main render -----
const std::vector<uint32_t>& SketchApp::render(){
    // wrap yaw to keep it bounded
    if (yaw_deg_ >  360.f) yaw_deg_ = std::fmod(yaw_deg_, 360.f);
    if (yaw_deg_ < -360.f) yaw_deg_ = std::fmod(yaw_deg_, 360.f);

    // camera (yaw=0 -> +Z; pitch=0 level)
    auto d2r=[](float d){ return d*3.1415926535f/180.f; };
    float ya=d2r(yaw_deg_), pi=d2r(pitch_deg_);
    tmx::vec3 eye{ std::sin(ya)*std::cos(pi)*dist_,
                   std::sin(pi)*dist_,
                   std::cos(ya)*std::cos(pi)*dist_ };
    V_ = tmx::lookAt(eye, tmx::vec3{0,0,0}, tmx::vec3{0,1,0});
    pipe_.setModel(M_); pipe_.setView(V_); pipe_.setProj(P_);

    // 1) parallel transform
    pipe_.transformIntoParallel(pos_, clip_, threads_?threads_:par::hw_threads());

    // 2) map to screen + NDC outcodes
    scr_.resize(clip_.size()); codes_.resize(clip_.size());
    par::parallel_for(0, clip_.size(), [&](std::size_t i){
        const auto& p = clip_[i];             // assume already divided by w in Pipeline3D
        uint8_t c=0;
        if(p.x<-1.f) c|=1; if(p.x>1.f) c|=2;
        if(p.y<-1.f) c|=4; if(p.y>1.f) c|=8;
        if(p.z< 0.f) c|=16; if(p.z>1.f) c|=32; // NDC z
        codes_[i]=c;
        int x=int((p.x*0.5f+0.5f)*float(W_));
        int y=int((1.f-(p.y*0.5f+0.5f))*float(H_));
        scr_[i]={x,y};
    }, threads_?threads_:par::hw_threads());

    // 3) clear buffers every frame (no accumulation)
    rast_.resize(W_,H_);
    rast_.clear({255,255,255,255});
    if ((int)zbuf_.size() != W_*H_) zbuf_.assign((size_t)W_*H_, 1.0f);
    if (faces_) buildDepth();  // hidden-line z prepass

    // 4) draw edges with depth test (hidden-line)
    auto drawLineDepth = [&](tmx::ivec2 a, float za, tmx::ivec2 b, float zb){
        int x0=a.x, y0=a.y, x1=b.x, y1=b.y;
        int dx=std::abs(x1-x0), sx=x0<x1?1:-1;
        int dy=-std::abs(y1-y0), sy=y0<y1?1:-1;
        int err=dx+dy;

        int steps = std::max(dx, -dy); steps = steps>0? steps : 1;
        int step  = 0;
        while(true){
            if((unsigned)x0<(unsigned)W_ && (unsigned)y0<(unsigned)H_){
                float t = float(step)/float(steps);
                float z = za*(1.f - t) + zb*t;           // NDC z along edge
                bool front = !faces_ || (z <= zbuf_[(size_t)y0*W_ + x0] + 1e-4f);
                if (front)        rast_.setPixelUnsafe(x0,y0,{0,0,0,255});        // visible
                else if(hidden_style_==HiddenStyle::Faint) rast_.setPixelUnsafe(x0,y0,{170,170,170,255}); // hidden faint
            }
            if(x0==x1 && y0==y1) break;
            int e2 = 2*err;
            if(e2>=dy){ err += dy; x0 += sx; }
            if(e2<=dx){ err += dx; y0 += sy; }
            ++step;
        }
    };

    for (const auto& e : edges_){
        if ((codes_[e.first] & codes_[e.second]) != 0) continue; // trivially outside
        auto a = scr_[e.first], b = scr_[e.second];
        int dx=a.x-b.x, dy=a.y-b.y; if(dx*dx+dy*dy < 4) continue; // tiny LOD
        drawLineDepth(a, clip_[e.first].z, b, clip_[e.second].z);
    }

    // 5) pack ARGB
    const auto& buf = rast_.pixels(); out_.resize(buf.size());
    for(size_t i=0;i<buf.size();++i){
        const auto& c = buf[i];
        out_[i] = (uint32_t(c.a)<<24)|(uint32_t(c.r)<<16)|(uint32_t(c.g)<<8)|uint32_t(c.b);
    }
    return out_;
}
