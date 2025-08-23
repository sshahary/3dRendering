# 3D Wireframe Renderer

Two executables:
- `render-gui` shows an OBJ in a window with simple camera controls
- `render-to-file` renders an OBJ to a PNG from a given camera

## Features
- OBJ load and edge extraction
- Perspective and orthographic projection
- MVP pipeline and NDC clip
- Hidden line aware wireframe with a z buffer
- Per vertex color gradients or monochrome black mode
- Multi thread tiling for faster line drawing
- GUI orbit controls with mouse and wheel
- CLI camera from coordinates with PNG output

## Build

### Requirements
- CMake 3.20 or newer
- C++17 compiler
- Qt 6 Widgets and Gui

### Configure and build
```bash
# from the project root
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
