#include "line_raster.hpp"
#include <cmath>

LineRaster::LineRaster(int w, int h) : w_(w), h_(h), buf_(size_t(w) * h) {}
void LineRaster::resize(int w, int h) {
  w_ = w;
  h_ = h;
  buf_.assign(size_t(w) * h, RGBA{});
}
void LineRaster::clear(const RGBA &c) {
  std::fill(buf_.begin(), buf_.end(), c);
}

// Bresenham solid line
void LineRaster::line(const tmx::ivec2 &a, const tmx::ivec2 &b, const RGBA &c) {
  int x0 = a.x, y0 = a.y, x1 = b.x, y1 = b.y;
  bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }
  int dx = x1 - x0;
  int dy = std::abs(y1 - y0);
  int err = dx / 2;
  int y = y0;
  int ystep = (y0 < y1) ? 1 : -1;
  for (int x = x0; x <= x1; ++x) {
    int px = steep ? y : x;
    int py = steep ? x : y;
    if (inBounds(px, py))
      set(px, py, c);
    err -= dy;
    if (err < 0) {
      y += ystep;
      err += dx;
    }
  }
}

static inline unsigned char lerp8(unsigned char a, unsigned char b, float t) {
  return (unsigned char)std::lround(float(a) + t * (float(b) - float(a)));
}

// NEW: Gradient color line with per-pixel depth test
void LineRaster::lineGradientDepth(const tmx::ivec2 &a, const tmx::ivec2 &b,
                                   const RGBA &ca, const RGBA &cb, float za,
                                   float zb, std::vector<float> &zbuf) {
  int x0 = a.x, y0 = a.y, x1 = b.x, y1 = b.y;
  bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  bool swapped = false;
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
    swapped = true;
  }

  RGBA c0 = swapped ? cb : ca;
  RGBA c1 = swapped ? ca : cb;
  float z0 = swapped ? zb : za;
  float z1 = swapped ? za : zb;

  int dx = x1 - x0;
  int dy = std::abs(y1 - y0);
  int err = dx / 2;
  int y = y0;
  int ystep = (y0 < y1) ? 1 : -1;

  const float invN = dx > 0 ? 1.0f / float(dx) : 1.0f;

  for (int x = x0; x <= x1; ++x) {
    float t = float(x - x0) * invN;
    RGBA c{lerp8(c0.r, c1.r, t), lerp8(c0.g, c1.g, t), lerp8(c0.b, c1.b, t),
           lerp8(c0.a, c1.a, t)};
    float z = z0 + t * (z1 - z0);

    int px = steep ? y : x;
    int py = steep ? x : y;

    if (inBounds(px, py)) {
      size_t idx = size_t(py) * size_t(w_) + size_t(px);
      if (z < zbuf[idx]) { // nearer wins
        zbuf[idx] = z;
        set(px, py, c);
      }
    }
    err -= dy;
    if (err < 0) {
      y += ystep;
      err += dx;
    }
  }
}
