#include "sketch_app.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <thread>  
#include "parallel.hpp"


// --- Helper: clip against near plane (z + w >= 0) and NDC box, then map to screen ---
static inline bool clipNearAndNDCToScreen(const tmx::vec4& A, const tmx::vec4& B,
            int W, int H, tmx::ivec2& SA, tmx::ivec2& SB)
{
    auto insideNear = [](const tmx::vec4& p){ return (p.z + p.w) >= 0.0f; }; // z >= -w
    bool a_in = insideNear(A), b_in = insideNear(B);
    if (!a_in && !b_in) return false;

    // Intersect with near plane if needed (z + w = 0)
    tmx::vec4 a = A, b = B;
    if (a_in ^ b_in) {
        const float denom = ((B.z + B.w) - (A.z + A.w));
        if (denom == 0.0f) return false;
        const float t = (-(A.z + A.w)) / denom;
        tmx::vec4 I{
            A.x + t * (B.x - A.x),
            A.y + t * (B.y - A.y),
            A.z + t * (B.z - A.z),
            A.w + t * (B.w - A.w)
        };
        if (!a_in) a = I; else b = I;
    }

    // Guard against division by ~0
    const float epsw = 1e-6f;
    if (std::fabs(a.w) < epsw || std::fabs(b.w) < epsw) return false;

    // To NDC
    float x0 = a.x / a.w, y0 = a.y / a.w;
    float x1 = b.x / b.w, y1 = b.y / b.w;

    // Liang–Barsky clip to NDC box [-1,1]x[-1,1]
    float dx = x1 - x0, dy = y1 - y0;
    float t0 = 0.0f, t1 = 1.0f;
    auto clipT = [&](float p, float q)->bool {
        if (p == 0.0f) return q >= 0.0f;
        float r = q / p;
    if (p < 0.0f) { if (r > t1) return false; if (r > t0) t0 = r; }
    else          { if (r < t0) return false; if (r < t1) t1 = r; }
    return true;
    };
    if (!clipT(-dx,  x0 + 1.0f)) return false; // left
    if (!clipT( dx,  1.0f - x0)) return false; // right
    if (!clipT(-dy,  y0 + 1.0f)) return false; // bottom
    if (!clipT( dy,  1.0f - y0)) return false; // top

    // Apply t0,t1 and map to screen
    float nx0 = x0 + t0 * dx, ny0 = y0 + t0 * dy;
    float nx1 = x0 + t1 * dx, ny1 = y0 + t1 * dy;
    SA = { int((nx0 * 0.5f + 0.5f) * float(W)),
    int((1.0f - (ny0 * 0.5f + 0.5f)) * float(H)) };
    SB = { int((nx1 * 0.5f + 0.5f) * float(W)),
    int((1.0f - (ny1 * 0.5f + 0.5f)) * float(H)) };
    return true;
}

// conservative triangle depth fill (screen space); z in [0,1]
static inline void fillDepthTri(const tmx::ivec2& a, float za,
    const tmx::ivec2& b, float zb,
    const tmx::ivec2& c, float zc,
    int W, int H, std::vector<float>& zbuf)
{
    int minx = std::max(0, std::min({a.x,b.x,c.x}));
    int maxx = std::min(W-1, std::max({a.x,b.x,c.x}));
    int miny = std::max(0, std::min({a.y,b.y,c.y}));
    int maxy = std::min(H-1, std::max({a.y,b.y,c.y}));
    if (minx>maxx || miny>maxy) return;
    auto edge = [](const tmx::ivec2& p, const tmx::ivec2& q, int x, int y){
    return (x - q.x)*(p.y - q.y) - (y - q.y)*(p.x - q.x);
    };

    const float area = float(edge(a,b,c.x,c.y));
    if (area == 0.f) return;
    for (int y=miny; y<=maxy; ++y){
        for (int x=minx; x<=maxx; ++x){
            float w0 = float(edge(b,c,x,y));
            float w1 = float(edge(c,a,x,y));
            float w2 = float(edge(a,b,x,y));
            // top-left rule
            if ((w0>0 || (w0==0 && ((b.y==c.y && b.x<c.x) || (b.y<c.y)))) &&
                (w1>0 || (w1==0 && ((c.y==a.y && c.x<a.x) || (c.y<a.y)))) &&
                (w2>0 || (w2==0 && ((a.y==b.y && a.x<b.x) || (a.y<b.y))))) {
                w0/=area; w1/=area; w2/=area;
                float z = w0*za + w1*zb + w2*zc;
                float& dst = zbuf[(size_t)y*W + x];
                if (z < dst) dst = z;
            }
        }
    }
}

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

tmx::vec3 SketchApp::computeCenter(const std::vector<tmx::vec3>& v){
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
P_(tmx::perspective(60.f, float(W_) / float(H_), 0.1f, 1000.f)),
pipe_(M_,V_,P_), vp_(W,H), rast_(W,H),
pos_(positions), edges_(edges)
{
// Auto-fit so model is fully visible on first frame
if(!positions.empty()){
tmx::vec3 mn = positions[0], mx = positions[0];
for(const auto& p: positions){
mn.x = std::min(mn.x, p.x); mn.y = std::min(mn.y, p.y); mn.z = std::min(mn.z, p.z);
mx.x = std::max(mx.x, p.x); mx.y = std::max(mx.y, p.y); mx.z = std::max(mx.z, p.z);
}
tmx::vec3 ext{ mx.x-mn.x, mx.y-mn.y, mx.z-mn.z };
float radius = 0.5f * std::max(std::max(ext.x, ext.y), ext.z);
const float fov_rad = 60.0f * 3.1415926535f / 180.0f; 
dist_ = std::max(dist_, radius / std::tan(fov_rad * 0.5f) * 1.2f);
}
}


void SketchApp::orbit(float dx, float dy){
    yaw_deg_   += dx;
    pitch_deg_ += dy;
    if (pitch_deg_ >  179.0f) pitch_deg_ =  179.0f;
    if (pitch_deg_ < -179.0f) pitch_deg_ = -179.0f;
}
void SketchApp::dolly(float factor){
    dist_ *= factor;
    if(dist_<0.2f) dist_=0.2f;
    if(dist_>2000.f) dist_=2000.f;
}
void SketchApp::resize(int W,int H){ W_=W; H_=H; vp_.resize(W,H); }

const std::vector<uint32_t>& SketchApp::render() {
    // 0) projection this frame
    const float aspect = float(W_) / float(H_ ? H_ : 1);
    P_ = tmx::perspective(60.f, aspect, 0.1f, 1000.f);
    pipe_.setProj(P_);

    // keep yaw bounded
    if (yaw_deg_ >  360.f) yaw_deg_ = std::fmod(yaw_deg_, 360.f);
    if (yaw_deg_ < -360.f) yaw_deg_ = std::fmod(yaw_deg_, 360.f);

    // 1) build view from orbit params
    auto d2r = [](float d){ return d * 3.1415926535f / 180.0f; };
    const float cy = d2r(yaw_deg_);
    const float cp = d2r(pitch_deg_);
    const tmx::vec3 eye{
        std::cos(cp) * std::cos(cy) * dist_,
        std::sin(cp) * dist_,
        std::cos(cp) * std::sin(cy) * dist_
    };
    const tmx::vec3 up = (std::cos(cp) >= 0.0f) ? tmx::vec3{0,1,0} : tmx::vec3{0,-1,0};

    V_ = tmx::lookAt(eye, tmx::vec3{0,0,0}, up);
    pipe_.setView(V_);
    pipe_.setModel(M_);

    // 2) vertices to CLIP SPACE (no divide in Pipeline3D)
    pipe_.transformIntoParallel(pos_, clip_, threads_ ? threads_ : par::hw_threads());

    // 3) per-vertex screen coords, clip codes, and depth in [0,1]
    scr_.resize(clip_.size());
    codes_.resize(clip_.size());
    z01_.resize(clip_.size());

    par::parallel_for(0, clip_.size(), [&](std::size_t i){
        const auto p = clip_[i];
        const float iw = p.w != 0.f ? 1.f / p.w : 0.f;
        const float nx = p.x * iw;
        const float ny = p.y * iw;
        const float nz = p.z * iw;
        z01_[i] = nz * 0.5f + 0.5f;

        uint8_t c = 0;
        if(nx < -1.f) c |= 1; if(nx >  1.f) c |= 2;
        if(ny < -1.f) c |= 4; if(ny >  1.f) c |= 8;
        if(z01_[i] < 0.f) c |= 16; if(z01_[i] > 1.f) c |= 32;
        codes_[i] = c;

        const int x = int((nx * 0.5f + 0.5f) * float(W_));
        const int y = int((1.f - (ny * 0.5f + 0.5f)) * float(H_));
        scr_[i] = { x, y };
    }, threads_ ? threads_ : par::hw_threads());

    // 4) optional depth fill for faces
    if (faces_ && !faces_->empty()) {
        clearDepth();
        for (const auto& f : *faces_) {
            if (f.indices.size() < 3) continue;
            int i0 = f.indices[0];
            auto p0 = scr_[i0]; float z0 = z01_[i0];
            for (size_t k = 1; k + 1 < f.indices.size(); ++k) {
                int i1 = f.indices[k], i2 = f.indices[k + 1];
                auto p1 = scr_[i1]; float z1 = z01_[i1];
                auto p2 = scr_[i2]; float z2 = z01_[i2];
                fillDepthTri(p0, z0, p1, z1, p2, z2, W_, H_, zbuf_);
            }
        }
    }

    // 5) build tiles and bin edges using NEAR+NDC clipped endpoints
    constexpr int TILE = 64;
    int gridW = (W_ + TILE - 1) / TILE;
    int gridH = (H_ + TILE - 1) / TILE;

    struct Tile { int x0,y0,x1,y1; std::vector<int> e; };
    std::vector<Tile> tiles(gridW * gridH);
    for (int ty = 0; ty < gridH; ++ty)
    for (int tx = 0; tx < gridW; ++tx) {
        auto& t = tiles[ty * gridW + tx];
        t.x0 = tx * TILE; t.y0 = ty * TILE;
        t.x1 = std::min(t.x0 + TILE - 1, W_ - 1);
        t.y1 = std::min(t.y0 + TILE - 1, H_ - 1);
    }

    const int minPix2 = int(min_len_px_ * min_len_px_);
    size_t added = 0;
    for (size_t i = 0; i < edges_.size() && added < max_edges_; i += edge_stride_) {
        const auto& e = edges_[i];

        tmx::ivec2 A,B;
        if (!clipNearAndNDCToScreen(clip_[e.first], clip_[e.second], W_, H_, A, B))
            continue;

        int dx = A.x - B.x, dy = A.y - B.y;
        if (dx*dx + dy*dy < minPix2) continue;

        int minx = std::min(A.x, B.x), miny = std::min(A.y, B.y);
        int maxx = std::max(A.x, B.x), maxy = std::max(A.y, B.y);
        int tx0 = std::clamp(minx / TILE, 0, gridW - 1);
        int ty0 = std::clamp(miny / TILE, 0, gridH - 1);
        int tx1 = std::clamp(maxx / TILE, 0, gridW - 1);
        int ty1 = std::clamp(maxy / TILE, 0, gridH - 1);
        for (int ty = ty0; ty <= ty1; ++ty)
        for (int tx = tx0; tx <= tx1; ++tx)
            tiles[ty * gridW + tx].e.push_back((int)i);

        ++added;
    }

    // 6) raster lines per tile (clip again to tile rect)
    rast_.resize(W_, H_);
    rast_.clear({255,255,255,255});

    auto worker = [&](int t0, int t1){
        for (int ti = t0; ti < t1; ++ti) {
            const auto& T = tiles[ti];
            for (int idx : T.e) {
                const auto& e = edges_[idx];

                tmx::ivec2 A,B;
                if (!clipNearAndNDCToScreen(clip_[e.first], clip_[e.second], W_, H_, A, B))
                    continue;

                int x0,y0,x1,y1;
                if (clipLineRect(A.x, A.y, B.x, B.y, T.x0, T.y0, T.x1, T.y1, x0, y0, x1, y1))
                    rast_.line({x0,y0}, {x1,y1}, {0,0,0,255});
            }
        }
    };

    const int Tn = (int)(threads_ ? threads_ : par::hw_threads());
    const int Ntiles = (int)tiles.size();
    const int per = (Ntiles + Tn - 1) / Tn;
    std::vector<std::thread> ts; ts.reserve(Tn);
    for (int t = 0; t < Tn; ++t) {
        int s = t * per, e = std::min(Ntiles, s + per);
        if (s < e) ts.emplace_back(worker, s, e);
    }
    for (auto& th : ts) th.join();

    // 7) pack ARGB
    const auto& buf = rast_.pixels(); out_.resize(buf.size());
    for (size_t i = 0; i < buf.size(); ++i) {
        const auto& c = buf[i];
        out_[i] = (uint32_t(c.a) << 24) | (uint32_t(c.r) << 16)
                | (uint32_t(c.g) << 8)  |  uint32_t(c.b);
    }
    return out_;
}
