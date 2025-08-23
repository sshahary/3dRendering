#pragma once
#include "camera.hpp"
#include "line_raster.hpp"
#include "mesh_loader.hpp"
#include "pipeline3d.hpp"
#include "tinymath.hpp"
#include "viewport.hpp"
#include <cstdint>
#include <utility>
#include <vector>

struct RGBA;

class SketchApp {
public:
  SketchApp(const std::vector<tmx::vec3> &positions,
            const std::vector<std::pair<int, int>> &edges, int W, int H);

  // main draw – returns ARGB32 buffer sized W×H
  const std::vector<uint32_t> &render();

  // interaction
  void orbit(float dx_deg, float dy_deg);
  void dolly(float factor);
  void resize(int W, int H);
  void setEdgeStride(int s) { edge_stride_ = s < 1 ? 1 : s; }
  void setThreads(unsigned n) { threads_ = n ? n : 1; }
  void setMinEdgePixels(float px) { min_len_px_ = px < 0.f ? 0.f : px; }
  void setMaxEdgesPerFrame(size_t n) { max_edges_ = n ? n : SIZE_MAX; }

  void cycleVertexColors();
  void setUseOrtho(bool v) { useOrtho_ = v; }
  void setCameraPosition(float x, float y, float z);
  void clearExplicitCamera();
  void setMonochrome(bool on);
  void toggleMonochrome();

  // hidden-line
  void setFaces(const std::vector<PolyFace> &f) { faces_ = &f; }
  void clearFaces() { faces_ = nullptr; }

private:
  // viewport
  int W_, H_;

  bool useOrtho_ = false;
  int color_seed_ = 0;
  float yaw_deg_ = 0.f;
  float pitch_deg_ = 0.f;
  float dist_ = 6.f;

  bool mono_ = false;

  // model center
  tmx::vec3 center_{};

  // matrices/pipeline
  tmx::mat4 M_, V_, P_;
  Pipeline3D pipe_;
  Viewport vp_;
  LineRaster rast_;

  Camera camera_;
  bool explicit_cam_ = false;
  tmx::vec3 cam_pos_{0.f, 0.f, 5.f};

  // output ARGB
  std::vector<uint32_t> out_;

  // geometry (positions by ref; edges local so we can stride/dedupe if needed)
  const std::vector<tmx::vec3> &pos_;
  std::vector<std::pair<int, int>> edges_;

  // per-frame temporaries
  std::vector<tmx::vec4> clip_; // NDC (x,y ∈ [-1,1], z ∈ [0,1])
  std::vector<float> z01_;      // NDC depth in [0,1] (for depth compare)
  std::vector<tmx::ivec2> scr_; // screen coords
  std::vector<uint8_t> codes_;  // outcodes

  // options
  int edge_stride_ = 1;
  unsigned threads_ = 0;        // 0 → hardware_concurrency
  float min_len_px_ = 1.0f;     // LOD threshold in pixels
  size_t max_edges_ = SIZE_MAX; // cap per frame

  // faces for hidden-line
  const std::vector<PolyFace> *faces_ = nullptr;
  std::vector<float> zbuf_; // size W_*H_, NDC depth in [0,1]; 1.0f = far
  void clearDepth() { zbuf_.assign((size_t)W_ * H_, 1.0f); }

  static tmx::vec3 computeCenter(const std::vector<tmx::vec3> &v);
  std::vector<RGBA> vcolor_; // per-vertex colors
  void buildDefaultVertexColors();
};
