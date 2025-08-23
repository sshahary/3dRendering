```markdown
# Report

## Overview
This project is a small 3D **wireframe renderer** for Wavefront OBJ files with two front-ends:
- **render-gui**: an interactive Qt6 viewer.
- **render-to-file**: a command-line tool that saves a PNG from a given camera.

When a model is loaded, its geometric center is translated to the origin `(0,0,0)` so the object is placed at the origin as required.

## Renderer core
The core follows a simple graphics pipeline. The **model** matrix recenters the mesh. The **view** matrix comes either from an orbit camera (yaw, pitch, distance) or from explicit camera coordinates; both look at the origin with +Y up. The **projection** can be **perspective** or **orthographic**.

Vertices are transformed to clip space, checked against the **near plane** (`z + w ≥ 0`), divided by `w`, and edges are clipped to the NDC box `[-1,1]^2` using a Liang–Barsky style routine. NDC is mapped to pixels via a viewport transform.

For visibility, the renderer keeps a screen-space **z-buffer**. Faces (if provided) can be conservatively depth-filled first, then edges are drawn with a per-pixel depth test so hidden lines are suppressed. Lines support **per-vertex color gradients** or **monochrome** (black on white) for print-style output.

To stay responsive, the screen is split into tiles (e.g., 64×64). Each tile is processed in parallel with `std::thread`, which reduces contention and avoids seam artifacts.

## GUI and CLI
The GUI is a Qt `QWidget` that renders on `paintEvent`. Controls:
- Mouse drag: **orbit**, wheel: **dolly**
- **O**: perspective/orthographic, **B**: monochrome toggle
- **Space**: cycle vertex colors, **C**: enter camera `(x,y,z)`

The CLI reuses the same core:
```

render-to-file \<input.obj> <cx> <cy> <cz> \<perspective|orthographic> \<output.png> \[-b|--black]

```
PNG output uses **QImage**.

## Design choices and C++ features
The design aims for clarity: a reusable **renderer_core** library used by both apps; Qt covers windowing and PNG IO. The code uses RAII, small value-type math structs, `std::vector` everywhere, lambdas, and **multithreading** via `std::thread`. This keeps the implementation compact, portable, and easy to test. Future work could add MSAA, proper hidden-line removal for all edges, and more robust face handling.
```
