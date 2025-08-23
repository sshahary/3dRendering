# Report — 3D Wireframe Renderer

**Author:** Shaharyar  
**Date:** 23.08.2025

**Abstract —** This project implements a modular, CPU-based **wireframe renderer** for Wavefront OBJ models with two front-ends: an interactive Qt GUI and a command-line tool that renders to PNG from a specified camera. The report follows a concise, sectioned layout similar to a peer example for clarity. :contentReference[oaicite:0]{index=0}

---

## 1. Core Renderer Implementation
The renderer is organized into small, reusable C++ components:

- **TinyMath:** Lightweight vector/matrix types and helpers used across the pipeline.
- **OBJ Loader:** Parses positions/indices and extracts a deduplicated edge list. On load, the mesh is **recentered** so its geometric center maps to `(0,0,0)`.
- **Pipeline (MVP):** Model translates to origin; View derives from either **orbit** (yaw, pitch, distance) or **explicit camera** coordinates; Projection supports **perspective** and **orthographic**.
- **Clipping & Mapping:** Per-vertex clip-space transform, **near-plane** check (`z + w ≥ 0`), divide by `w`, then Liang–Barsky style clipping to the **NDC** box `[-1,1]^2` and viewport mapping to pixels.
- **Rasterizer:** Tiled, multi-threaded line drawing (Bresenham-style) with **per-vertex color gradients** and a screen-space **z-buffer**. Optional conservative triangle depth fill improves hidden-line behavior. A **monochrome** mode draws black lines on white for print-friendly output.

---

## 2. GUI Architecture
The GUI is a compact **Qt6** application (`QWidget`) that asks the core to render during `paintEvent`. Input handling provides:
- **Orbit** (mouse drag) and **dolly** (wheel/trackpad)
- Toggles: **O** (projection), **B** (monochrome), **Space** (cycle colors)
- **C** prompts to set camera `(x, y, z)` directly

This separation—controls in the widget, rendering in the core—keeps the codebase maintainable and testable. :contentReference[oaicite:1]{index=1}

---

## 3. Exporting Rendered Images
The CLI front-end reuses the same core and writes PNGs via **Qt’s `QImage`**:
```

render-to-file \<input.obj> <cx> <cy> <cz> \<perspective|orthographic> \<output.png> \[-b|--black]

```
Environment variables `RTF_WIDTH/RTF_HEIGHT` allow custom output sizes.

---

## 4. Software Design & C++ Features
- **Modularity:** A single **renderer_core** library powers both executables; UI and rendering stay decoupled for flexibility. :contentReference[oaicite:2]{index=2}  
- **Determinism & Simplicity:** NDC-space clipping and orthographic mode simplify testing and behavior.
- **Performance:** Coarse **tiling** plus `std::thread` parallelism scales on multi-core CPUs; contiguous pixel/z buffers aid cache locality.
- **C++ Practices:** RAII, const-correct value types, `std::vector` throughout, lambdas for worker tasks, and clear compile flags (`-Wall -Wextra`, optional `-Werror`).  
- **Tooling:** CMake builds both apps; tests (Catch2) validate core behavior; `clang-format` keeps style consistent.

_This structure delivers a compact, portable renderer that meets the assignment requirements while remaining easy to extend (e.g., MSAA, stronger hidden-line removal, alternative image backends)._
