#pragma once
#include <vector>
#include <utility>
#include <cstdint>
#include "tinymath.hpp"
#include "pipeline3d.hpp"
#include "viewport.hpp"
#include "line_raster.hpp"

class SketchApp {
public:
    SketchApp(const std::vector<tmx::vec3>& positions,
              const std::vector<std::pair<int,int>>& edges,
              int W, int H);

    const std::vector<uint32_t>& render();

    void orbit(float dx_deg, float dy_deg);
    void dolly(float factor);
    void resize(int W,int H);
    void setEdgeStride(int s)           { edge_stride_ = s < 1 ? 1 : s; }
    void setThreads(unsigned n)         { threads_ = n ? n : 1; }
    void setMinEdgePixels(float px)     { min_len_px_ = px < 0.f ? 0.f : px; }
    void setGrid(int px)                { grid_px_ = px < 1 ? 0 : px; }
    void setMaxEdgesPerFrame(size_t n)  { max_edges_ = n ? n : SIZE_MAX; }
    void usePerspective(bool on);

private:
    int W_, H_;

    float yaw_deg_   = 0.f;   // yaw=0 -> +Z (front view)
    float pitch_deg_ = 0.f;   // level
    float dist_      = 6.f;   // distance

    tmx::vec3 center_{};

    // matrices/pipeline
    tmx::mat4 M_, V_, P_;
    Pipeline3D pipe_;
    Viewport   vp_;
    LineRaster rast_;
    std::vector<uint32_t> out_;

    const std::vector<tmx::vec3>& pos_;
    std::vector<std::pair<int,int>> edges_;

    std::vector<tmx::vec4>  ndc_;
    std::vector<tmx::ivec2> scr_;
    std::vector<uint8_t>    codes_;

    // macro-grid overdraw limiter
    std::vector<uint8_t> gridOcc_;
    int gridW_ = 0, gridH_ = 0;

    // options
    int        edge_stride_ = 1;
    unsigned   threads_     = 0;        // 0 → hardware_concurrency
    float      min_len_px_  = 1.0f;     // LOD threshold in pixels
    int        grid_px_     = 0;        // 0 = off
    size_t     max_edges_   = SIZE_MAX; // cap per frame

    // helpers
    static tmx::vec3 computeCenter(const std::vector<tmx::vec3>& v);
};
