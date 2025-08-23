<!-- @format -->

# Report

## Overview

This project implements a 3D **wireframe** renderer for Wavefront OBJ models with two front-ends:

- **render-gui**: an interactive Qt6 viewer.
- **render-to-file**: a headless renderer writing a PNG from a user-specified camera.

On load, the mesh is translated so its geometric **center** sits at the origin `(0,0,0)`, satisfying the “object at the origin” requirement.

## Renderer core

The core follows a classic MVP pipeline. The model matrix **M** translates the mesh center to the origin. The view matrix **V** comes either from an **orbit** camera (yaw/pitch/dist) or from an **explicit** camera position set by coordinates; a small `Camera` class builds the view via `lookAt`. The projection **P** is either **perspective** or **orthographic** (see `Projection`).

Vertices are transformed to clip space and clipped against the **near plane** (`z + w ≥ 0`). Edges are then clipped to the **NDC box** using a Liang–Barsky variant before mapping to screen coordinates. For hidden-line awareness, the renderer maintains a **z-buffer** in screen space. Faces (if present) can be conservatively depth-filled first; then edges are drawn with per-pixel depth tests so lines behind geometry are suppressed.

Rasterization uses a Bresenham-style **line drawer** extended with **per-vertex color gradients** and a depth compare. The screen is divided into **tiles** (e.g., 64×64); each tile’s edge list is processed in parallel with `std::thread`, which reduces contention and keeps seams tight. A **monochrome** mode draws black lines on white for print-friendly output.

## GUI and CLI

The GUI is a small Qt `QWidget` that asks the core to render on each paint. Interactions:

- **Mouse drag**: orbit; **wheel**: dolly.
- **O**: toggle perspective/orthographic; **B**: toggle monochrome.
- **Space**: cycle vertex colors; **C**: input camera `(x,y,z)`.

The CLI reuses the same core:

```

render-to-file \<input.obj> <cx> <cy> <cz> \<perspective|orthographic> \<output.png> \[-b|--black]

```

PNG writing uses Qt’s `QImage`.

## Design choices and C++ features

The code favors **clarity and portability**: a reusable `renderer_core` library shared by both apps; Qt provides both windowing and PNG IO to avoid extra dependencies. Key C++ features used include RAII, small value-type math structs, `std::vector` throughout, lambdas, and **multithreading** via `std::thread`. The clipping helpers and tiled raster keep behavior deterministic and testable while remaining efficient for medium meshes.

```

```
