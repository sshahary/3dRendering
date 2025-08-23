#include "controls.hpp"
#include "mesh_loader.hpp"
#include "sketch_app.hpp"
#include <QApplication>
#include <QWidget>
#include <algorithm>
#include <thread>

int main(int argc, char **argv) {
  QApplication qapp(argc, argv);
  if (argc < 2) {
    qWarning("Usage: render-gui <input.obj>");
    return 1;
  }

  MeshLoader ml;
  if (!ml.loadOBJ(argv[1])) {
    qWarning("Failed to load OBJ");
    return 2;
  }

  SketchApp app(ml.positions, ml.edges, 1000, 1000);
  app.setThreads(std::max(1u, std::thread::hardware_concurrency()));

  SketchWidget w(&app);
  w.resize(1000, 1000);
  w.show();

  return qapp.exec();
}
