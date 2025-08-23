#pragma once
#include "tinymath.hpp"
#include <algorithm>
#include <cstdint>
#include <vector>

struct RGBA {
  unsigned char r, g, b, a;
  RGBA(unsigned char R = 0, unsigned char G = 0, unsigned char B = 0,
       unsigned char A = 255)
      : r(R), g(G), b(B), a(A) {}
};

class LineRaster {
public:
  LineRaster(int w, int h);
  void resize(int w, int h);
  void clear(const RGBA &c);
  void line(const tmx::ivec2 &a, const tmx::ivec2 &b, const RGBA &c);
  // NEW: vertex-color gradient with per-pixel depth test
  // za and zb are depths in [0,1], smaller = nearer
  void lineGradientDepth(const tmx::ivec2 &a, const tmx::ivec2 &b,
                         const RGBA &ca, const RGBA &cb, float za, float zb,
                         std::vector<float> &zbuf);

  const std::vector<RGBA> &pixels() const { return buf_; }
  int width() const { return w_; }
  int height() const { return h_; }

  void setPixelUnsafe(int x, int y, const RGBA &c) {
    if (x < 0 || y < 0 || x >= w_ || y >= h_)
      return;
    buf_[y * w_ + x] = c;
  }

private:
  int w_, h_;
  std::vector<RGBA> buf_;
  void set(int x, int y, const RGBA &c) { buf_[y * w_ + x] = c; }
  inline bool inBounds(int x, int y) const {
    return unsigned(x) < unsigned(w_) && unsigned(y) < unsigned(h_);
  }
};
