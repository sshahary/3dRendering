#include <QColorSpace>
#include <QImage>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "mesh_loader.hpp"
#include "sketch_app.hpp"

static void usage() {
  std::cerr << "Usage:\n"
               "  render-to-file <input.obj> <cam_x> <cam_y> <cam_z> "
               "<perspective|orthographic> <output.png> [-b|--black]\n"
               "\n"
               "Color vs black:\n"
               "  • Default (no -b/--black): render in color.\n"
               "  • With -b or --black: render in monochrome (black lines on "
               "white).\n"
               "\n"
               "Examples:\n"
               "  render-to-file input.obj -4 -3 2 perspective out.png         "
               " # color\n"
               "  render-to-file input.obj -4 -3 2 perspective out.png -b      "
               " # black\n";
}

static bool save_png_argb32(const std::string &path, int W, int H,
                            const std::vector<uint32_t> &argb) {
  QImage img(W, H, QImage::Format_ARGB32);
  for (int y = 0; y < H; ++y) {
    std::memcpy(img.scanLine(y),
                reinterpret_cast<const uint8_t *>(argb.data()) +
                    size_t(y) * W * 4,
                size_t(W) * 4);
  }
  img.setColorSpace(QColorSpace::SRgb);
  return img.save(QString::fromStdString(path));
}

int main(int argc, char **argv) {
  bool mono = false;
  if (argc == 8) {
    std::string f = argv[7];
    if (f == "-b" || f == "--black")
      mono = true;
    else {
      usage();
      return 1;
    }
  } else if (argc != 7) {
    usage();
    return 1;
  }
  const std::string inObj = argv[1];
  const float camX = std::strtof(argv[2], nullptr);
  const float camY = std::strtof(argv[3], nullptr);
  const float camZ = std::strtof(argv[4], nullptr);
  std::string proj = argv[5];
  const std::string outPng = argv[6];

  std::transform(proj.begin(), proj.end(), proj.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  const bool useOrtho =
      (proj == "orthographic") ? true
      : (proj == "perspective")
          ? false
          : (usage(), std::cerr << "Invalid projection token\n", exit(2),
             false);

  MeshLoader ml;
  if (!ml.loadOBJ(inObj)) {
    std::cerr << "Failed to load OBJ: " << inObj << "\n";
    return 3;
  }

  int W = 1000, H = 1000;
  if (const char *w = std::getenv("RTF_WIDTH"))
    W = std::max(16, std::atoi(w));
  if (const char *h = std::getenv("RTF_HEIGHT"))
    H = std::max(16, std::atoi(h));

  SketchApp app(ml.positions, ml.edges, W, H);
  app.setUseOrtho(useOrtho);
  app.setCameraPosition(camX, camY, camZ);
  app.setMonochrome(mono);

  const auto &pixels = app.render();
  if (!save_png_argb32(outPng, W, H, pixels)) {
    std::cerr << "Failed to write " << outPng << "\n";
    return 4;
  }
  std::cout << "Wrote " << outPng << "  " << W << "x" << H << "  camera("
            << camX << ", " << camY << ", " << camZ << ") "
            << (useOrtho ? "orthographic" : "perspective") << "\n";
  return 0;
}
