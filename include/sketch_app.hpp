#pragma once
#include <vector>
#include <utility>
#include <cstdint>
#include "tinymath.hpp"
#include "pipeline3d.hpp"
#include "viewport.hpp"
#include "line_raster.hpp"

enum class HiddenStyle { Off, Faint };
struct PolyFace;

class SketchApp {
public:
    SketchApp(const std::vector<tmx::vec3>& positions,
              const std::vector<std::pair<int,int>>& edges,
              int W, int H);

    const std::vector<uint32_t>& render();
    void orbit(float dx_deg, float dy_deg);
    void dolly(float factor);
    void resize(int W,int H);
    void setThreads(unsigned n){ threads_ = n? n : 1; }
    void setHiddenStyle(HiddenStyle s){ hidden_style_  = s; }
    void setFaces(const std::vector<PolyFace>& faces) { faces_ = &faces; }

private:
    int W_, H_;

    float yaw_deg_   = 0.f;   // yaw=0 -> +Z (front view)
    float pitch_deg_ = 0.f;   // level
    float dist_      = 6.f;   // distance

    tmx::mat4 M_, V_, P_;
    Pipeline3D pipe_;
    Viewport   vp_;
    LineRaster rast_;

    const std::vector<tmx::vec3>& pos_;
    const std::vector<std::pair<int,int>>& edges_;
    const std::vector<PolyFace>* faces_ = nullptr;

    std::vector<tmx::vec4>  clip_;
    std::vector<tmx::ivec2> scr_;
    std::vector<uint8_t>    codes_;

    std::vector<uint32_t> out_;
    std::vector<float> zbuf_;
    unsigned     threads_ = 0;
    HiddenStyle  hidden_style_  = HiddenStyle::Off;
    void buildDepth();
};
