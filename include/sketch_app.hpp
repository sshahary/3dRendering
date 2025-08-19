#pragma once
#include <vector>
#include <utility>
#include <cstdint>
#include "tinymath.hpp"
#include "pipeline3d.hpp"
#include "viewport.hpp"
#include "line_raster.hpp"
#include "mesh_loader.hpp"

enum class HiddenStyle { Off, Faint };

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

    void setFaces(const std::vector<PolyFace>& faces) { faces_ = &faces; }

    void setHiddenStyle(HiddenStyle s) { hidden_style_ = s; }

private:
    int W_, H_;
    // camera/orbit
    float yaw_deg_ = 35.f;  
    float pitch_deg_ = -20.f;
    float dist_ = 6.f;

    // model center (computed from bbox)
    tmx::vec3 center_{};

    // matrices and pipeline
    tmx::mat4 M_, V_, P_;
    Pipeline3D pipe_;
    Viewport   vp_;
    LineRaster rast_;

    // output ARGB
    std::vector<uint32_t> out_;

    // geometry (refs)
    const std::vector<tmx::vec3>& pos_;
    const std::vector<std::pair<int,int>>& edges_;

    // temporaries
    std::vector<tmx::vec4>  clip_;
    std::vector<tmx::ivec2> scr_;
    std::vector<uint8_t>    codes_;

    // threading
    unsigned threads_ = 0;

    const std::vector<PolyFace>* faces_ = nullptr;

    // Depth buffer for hidden-line
    std::vector<float> zbuf_;   // size W_*H_, NDC depth in [0,1]; 1.0f = far
    void clearDepth() { zbuf_.assign(W_*H_, 1.0f); }

    // Build depth from faces (triangulated)
    void buildDepth(); // implemented below

    // Hidden-line style
    HiddenStyle hidden_style_ = HiddenStyle::Off;
};
