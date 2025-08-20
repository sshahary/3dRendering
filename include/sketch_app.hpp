#pragma once
#include <vector>
#include <utility>
#include <cstdint>
#include "tinymath.hpp"
#include "pipeline3d.hpp"
#include "viewport.hpp"
#include "line_raster.hpp"
#include "mesh_loader.hpp"

enum class HiddenStyle { Off, Faint }; // draw occluded edges faintly

class SketchApp {
public:
    SketchApp(const std::vector<tmx::vec3>& positions,
              const std::vector<std::pair<int,int>>& edges,
              int W, int H);

    // main draw – returns ARGB32 buffer sized W×H
    const std::vector<uint32_t>& render();

    // interaction
    void orbit(float dx_deg, float dy_deg);  // 360° yaw (unbounded), clamped pitch
    void dolly(float factor);                // zoom
    void resize(int W,int H);                // viewport resize
    void usePerspective(bool on);

    // knobs (optional)
    void setEdgeStride(int s)           { edge_stride_ = s < 1 ? 1 : s; }
    void setThreads(unsigned n)         { threads_ = n ? n : 1; }
    void setMinEdgePixels(float px)     { min_len_px_ = px < 0.f ? 0.f : px; }
    void setGrid(int px)                { grid_px_ = px < 1 ? 0 : px; }    // 0 = off
    void setMaxEdgesPerFrame(size_t n)  { max_edges_ = n ? n : SIZE_MAX; }

    // hidden-line / faces (optional)
    void setFaces(const std::vector<PolyFace>& f) { faces_ = &f; }
    void setHidden(HiddenStyle s) { hidden_style_ = s; }

private:
    // viewport
    int W_, H_;

    // camera/orbit
    float yaw_deg_   = 0.f;   // yaw=0 → +Z (front)
    float pitch_deg_ = 0.f;   // level
    float dist_      = 6.f;   // distance (auto-fit at ctor)

    // model center
    tmx::vec3 center_{};

    // matrices/pipeline
    tmx::mat4 M_, V_, P_;
    Pipeline3D pipe_;
    Viewport   vp_;
    LineRaster rast_;

    // output ARGB
    std::vector<uint32_t> out_;

    // geometry (positions by ref; edges local so we can stride/dedupe if needed)
    const std::vector<tmx::vec3>& pos_;
    std::vector<std::pair<int,int>> edges_;

    // per-frame temporaries
    std::vector<tmx::vec4>  clip_;   // NDC (x,y ∈ [-1,1], z ∈ [0,1])
    std::vector<tmx::ivec2> scr_;    // screen coords
    std::vector<uint8_t>    codes_;  // outcodes

    // macro-grid overdraw limiter
    std::vector<uint8_t> gridOcc_;
    int gridW_ = 0, gridH_ = 0;

    // options
    int        edge_stride_ = 1;
    unsigned   threads_     = 0;          // 0 → hardware_concurrency
    float      min_len_px_  = 1.0f;       // LOD threshold in pixels
    int        grid_px_     = 0;          // 0 = off
    size_t     max_edges_   = SIZE_MAX;   // cap per frame

    // faces for hidden-line
    const std::vector<PolyFace>* faces_ = nullptr;
    HiddenStyle hidden_style_ = HiddenStyle::Off;
    std::vector<float> zbuf_;             // size W_*H_, NDC depth in [0,1]; 1.0f = far
    void clearDepth() { zbuf_.assign((size_t)W_*H_, 1.0f); }
    void buildDepth();                    // builds zbuf_ from faces (fan triangulate)

    // helpers
    static tmx::vec3 computeCenter(const std::vector<tmx::vec3>& v);
};
