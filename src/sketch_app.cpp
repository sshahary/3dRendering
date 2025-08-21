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
    M_(tmx::mat4::translation(-center_.x, -center_.y, -center_.z)),
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
    buildDefaultVertexColors();
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

// // --- math helpers (tiny, self-contained) ---
// static inline tmx::vec3 cross3(const tmx::vec3& a, const tmx::vec3& b){
//     return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
// }
// static inline float dot3(const tmx::vec3& a, const tmx::vec3& b){
//     return a.x*b.x + a.y*b.y + a.z*b.z;
// }
// static inline tmx::vec3 norm3(const tmx::vec3& v){
//     float L = std::sqrt(dot3(v,v)); return (L>0.f)? tmx::vec3{v.x/L, v.y/L, v.z/L} : tmx::vec3{0,0,1};
// }
// static inline tmx::vec3 xformPoint(const tmx::mat4& M, const tmx::vec3& p){
//     tmx::vec4 v{p.x,p.y,p.z,1.f}; auto r = M * v; return {r.x,r.y,r.z};
// }
// static inline tmx::vec3 xformDir(const tmx::mat4& M, const tmx::vec3& v){
//     tmx::vec4 w{v.x,v.y,v.z,0.f}; auto r = M * w; return tmx::vec3{r.x,r.y,r.z};
// }

// --- default vertex colors (from earlier step; keep yours if already added) ---
static inline unsigned char clampU8(int v){ return (unsigned char)std::clamp(v,0,255); }
void SketchApp::buildDefaultVertexColors(){
    vcolor_.resize(pos_.size());
    for (size_t i=0;i<pos_.size();++i){
        uint32_t h = uint32_t(i)*2654435761u; // Knuth hash
        unsigned char r = (h>>16)&0xFF, g=(h>>8)&0xFF, b=h&0xFF;
        r = clampU8(int(r)*3/2); g = clampU8(int(g)*3/2); b = clampU8(int(b)*3/2);
        vcolor_[i] = RGBA{r,g,b,255};
    }
}

// // --- build unique-edge adjacency & dihedral-filtered feature edges ---
// void SketchApp::buildAdjacencyAndFeatures(){
//     if (!faces_ || faces_->empty()) return;

//     // per-face normal + centroid (model space)
//     const size_t Fn = faces_->size();
//     face_normal_model_.assign(Fn, {0,0,1});
//     face_centroid_model_.assign(Fn, {0,0,0});

//     for (size_t fi=0; fi<Fn; ++fi){
//         const auto& f = (*faces_)[fi];
//         if (f.indices.size() < 3) continue;
//         const tmx::vec3 p0 = pos_[f.indices[0]];
//         const tmx::vec3 p1 = pos_[f.indices[1]];
//         const tmx::vec3 p2 = pos_[f.indices[2]];
//         face_normal_model_[fi] = norm3(cross3(p1-p0, p2-p0));

//         // centroid
//         tmx::vec3 c{0,0,0};
//         for(int id : f.indices) { c.x+=pos_[id].x; c.y+=pos_[id].y; c.z+=pos_[id].z; }
//         float inv = 1.0f / float(f.indices.size());
//         face_centroid_model_[fi] = { c.x*inv, c.y*inv, c.z*inv };
//     }

//     // map normalized edge (min,max) -> unique-edge index
//     std::unordered_map<uint64_t,int> map;
//     map.reserve(edges_.size()*2);
//     auto key = [](int a,int b)->uint64_t{ if(a>b) std::swap(a,b); return (uint64_t(uint32_t(a))<<32) | uint32_t(b); };

//     uedges_.clear(); uedges_.reserve(edges_.size());

//     for (size_t fi=0; fi<Fn; ++fi){
//         const auto& f = (*faces_)[fi];
//         const int n = int(f.indices.size());
//         for (int k=0; k<n; ++k){
//             int a = f.indices[k];
//             int b = f.indices[(k+1)%n];
//             uint64_t K = key(a,b);
//             auto it = map.find(K);
//             if (it == map.end()){
//                 UE ue; ue.v0 = std::min(a,b); ue.v1 = std::max(a,b); ue.f0 = int(fi);
//                 int idx = (int)uedges_.size(); map.emplace(K, idx); uedges_.push_back(ue);
//             } else {
//                 UE& ue = uedges_[it->second];
//                 if (ue.f1 == -1) ue.f1 = int(fi);
//             }
//         }
//     }

//     // dihedral selection
//     feature_edges_.clear(); feature_edges_.reserve(uedges_.size());
//     const float cosThresh = std::cos(dihedral_thresh_deg_ * 3.1415926535f / 180.f); // keep when angle >= thresh
//     for (const UE& ue : uedges_){
//         if (ue.f1 == -1) { // boundary always kept
//             feature_edges_.push_back({ue.v0, ue.v1});
//         } else {
//             const tmx::vec3 n0 = face_normal_model_[ue.f0];
//             const tmx::vec3 n1 = face_normal_model_[ue.f1];
//             float c = std::clamp(dot3(n0,n1), -1.0f, 1.0f);
//             // angle = acos(c). Keep if angle >= thresh  ↔  c <= cos(thresh)
//             if (c <= cosThresh) feature_edges_.push_back({ue.v0, ue.v1});
//         }
//     }

//     rebuildFeatures_ = false;
// }

// // --- per-frame: decide which edges to draw (features + optional silhouettes) ---
// void SketchApp::buildFrameEdges(const tmx::vec3& eye_world){
//     frame_edges_.clear();

//     const bool haveFaces = faces_ && !faces_->empty() && !uedges_.empty();

//     // If needed (first time, or dihedral changed) rebuild feature list
//     if (haveFaces && rebuildFeatures_) buildAdjacencyAndFeatures();

//     // Start from feature edges if requested; otherwise all mesh edges
//     if (haveFaces && use_feature_edges_only_) {
//         frame_edges_ = feature_edges_;
//     } else {
//         frame_edges_ = edges_;
//     }

//     // Add silhouettes: edges whose two adjacent faces have opposite facing
//     if (haveFaces && show_silhouette_) {
//         // compute front/back per face in WORLD space
//         face_front_.assign(faces_->size(), 0);
//         tmx::vec3 eye = eye_world;

//         // transform normals by M_ (direction only)
//         tmx::vec3 n0w;
//         for (size_t fi=0; fi<faces_->size(); ++fi){
//             tmx::vec3 c = xformPoint(M_, face_centroid_model_[fi]);
//             tmx::vec3 n = norm3(xformDir(M_, face_normal_model_[fi]));
//             float s = dot3(n, eye - c);   // >0 means front-facing
//             face_front_[fi] = (s > 0.f) ? 1 : 0;
//         }

//         for (const UE& ue : uedges_) {
//             if (ue.f0 >= 0 && ue.f1 >= 0){
//                 if (face_front_[ue.f0] ^ face_front_[ue.f1]) {
//                     frame_edges_.push_back({ue.v0, ue.v1});
//                 }
//             }
//         }
//         // (optional) de-duplicate silhouettes added on top of features:
//         std::sort(frame_edges_.begin(), frame_edges_.end(),
//                   [](auto& A, auto& B){ return (A.first==B.first)? (A.second<B.second):(A.first<B.first); });
//         frame_edges_.erase(std::unique(frame_edges_.begin(), frame_edges_.end()), frame_edges_.end());
//     }
// }

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

    // clip space vertices
    pipe_.transformIntoParallel(pos_, clip_, threads_ ? threads_ : par::hw_threads());

    // per-vertex screen coords, codes, and depth
    scr_.resize(clip_.size());
    codes_.resize(clip_.size());
    z01_.resize(clip_.size());

    par::parallel_for(0, clip_.size(), [&](std::size_t i){
        const auto p = clip_[i];
        const float iw = p.w != 0.f ? 1.f / p.w : 0.f;
        const float nx = p.x * iw;
        const float ny = p.y * iw;
        const float nz = p.z * iw;        // [-1,1]
        z01_[i] = nz * 0.5f + 0.5f;

        uint8_t c = 0;
        if(nx < -1.f) c |= 1; if(nx >  1.f) c |= 2;
        if(ny < -1.f) c |= 4; if(ny >  1.f) c |= 8;
        if(z01_[i] < 0.f) c |= 16; if(z01_[i] > 1.f) c |= 32;
        codes_[i] = c;

        const int x = int((nx*0.5f+0.5f)*float(W_));
        const int y = int((1.f-(ny*0.5f+0.5f))*float(H_));
        scr_[i] = {x,y};
    }, threads_ ? threads_ : par::hw_threads());

    // faces depth fill if present
    if (faces_ && !faces_->empty()) {
        clearDepth();
        for (const auto& f : *faces_) {
            if (f.indices.size() < 3) continue;
            int i0 = f.indices[0];
            auto p0 = scr_[i0]; float z0 = z01_[i0];
            for (size_t k=1; k+1 < f.indices.size(); ++k) {
                int i1 = f.indices[k], i2 = f.indices[k+1];
                auto p1 = scr_[i1]; float z1 = z01_[i1];
                auto p2 = scr_[i2]; float z2 = z01_[i2];
                fillDepthTri(p0, z0, p1, z1, p2, z2, W_, H_, zbuf_);
            }
        }
    } else {
        // ensure and clear zbuf for lines only
        if (zbuf_.size() != size_t(W_)*size_t(H_))
            zbuf_.assign(size_t(W_)*size_t(H_), 1.0f);
        else
            std::fill(zbuf_.begin(), zbuf_.end(), 1.0f);
    }

    // tiles with one-tile-per-edge to avoid seam gaps
    constexpr int TILE = 64;
    int gridW = (W_ + TILE - 1) / TILE;
    int gridH = (H_ + TILE - 1) / TILE;
    struct Tile { std::vector<int> e; };
    std::vector<Tile> tiles(gridW * gridH);

    const int minPix2 = int(min_len_px_ * min_len_px_);
    size_t added = 0;
    for (size_t i=0; i<edges_.size() && added < max_edges_; i += edge_stride_) {
        const auto& e = edges_[i];
        tmx::ivec2 A,B;
        if (!clipNearAndNDCToScreen(clip_[e.first], clip_[e.second], W_, H_, A, B))
            continue;

        int dx=A.x-B.x, dy=A.y-B.y;
        if (dx*dx + dy*dy < minPix2) continue;

        int mx = (A.x + B.x) >> 1;
        int my = (A.y + B.y) >> 1;
        int tx = std::clamp(mx / TILE, 0, gridW - 1);
        int ty = std::clamp(my / TILE, 0, gridH - 1);
        tiles[ty*gridW + tx].e.push_back(int(i));
        ++added;
    }

    rast_.resize(W_,H_);
    rast_.clear({255,255,255,255});

    auto worker = [&](int start, int end){
        for (int ti=start; ti<end; ++ti){
            for (int idx : tiles[ti].e){
                const auto& e = edges_[idx];
                tmx::ivec2 A,B;
                if (!clipNearAndNDCToScreen(clip_[e.first], clip_[e.second], W_, H_, A, B))
                    continue;

                const RGBA ca = vcolor_[e.first];
                const RGBA cb = vcolor_[e.second];
                const float za = z01_[e.first];
                const float zb = z01_[e.second];

                rast_.lineGradientDepth(A, B, ca, cb, za, zb, zbuf_);
            }
        }
    };

    const int Tn = int(threads_ ? threads_ : par::hw_threads());
    const int N  = int(tiles.size());
    const int per = (N + Tn - 1) / Tn;
    std::vector<std::thread> ts; ts.reserve(Tn);
    for (int t=0; t<Tn; ++t) {
        int s=t*per, e=std::min(N, s+per);
        if (s<e) ts.emplace_back(worker, s, e);
    }
    for (auto& th: ts) th.join();

    // pack ARGB
    const auto& buf = rast_.pixels(); out_.resize(buf.size());
    for (size_t i=0;i<buf.size();++i){
        const auto& c = buf[i];
        out_[i] = (uint32_t(c.a)<<24) | (uint32_t(c.r)<<16)
                | (uint32_t(c.g)<<8)  |  uint32_t(c.b);
    }
    return out_;
}
