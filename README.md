# 3D Wireframe Renderer

This project has two programs:

* **render-gui** – opens a window and shows a Wavefront **OBJ** with simple camera controls
* **render-to-file** – renders an OBJ to a **PNG** from a camera you pass on the command line

The mesh is recentered so its **geometric center** is at `(0, 0, 0)`. Both **perspective** and **orthographic** projections are supported.

---

## Features

* Load OBJ and extract edges
* Standard MVP pipeline (model → view → projection → NDC → screen)
* Near-plane and NDC clipping (Liang–Barsky for edges)
* Depth-aware wireframe (z-buffer), optional triangle depth fill
* Per-vertex color gradients or **black on white** (print mode)
* Multithreaded, tiled line rasterizer
* Qt6 GUI: orbit (mouse), dolly (wheel), quick keyboard toggles
* CLI writes PNG (via Qt’s QImage)

---

## Requirements

* **CMake ≥ 3.20**
* **C++17** compiler
* **Qt 6** (Widgets, Gui)

  * macOS: `brew install qt`
  * Ubuntu/Debian: `sudo apt install qt6-base-dev`
  * Windows: install Qt and pass `-DCMAKE_PREFIX_PATH=...`

Optional:

* **Catch2 v3** for tests (auto-fetched if missing)
* **clang-format** for formatting (`make format`)

---

## Build

### CMake

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

**macOS (Homebrew Qt):**

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(brew --prefix qt)"
cmake --build build
```

**Windows (MSVC):**

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="C:\Qt\6.7.2\msvc2019_64"
cmake --build build --config Release
```

> Binaries are written to the **repo root**: `./render-gui`, `./render-to-file`.

### Makefile (convenience)

* `make` – configure + build (Release)
* `make run OBJ=path/to/model.obj` – build then start **render-gui**
* `make debug` – Debug build
* `make asan` – Debug + AddressSanitizer (if enabled in CMake)
* `make clean` – remove `build/`
* `make distclean` – remove `build/` **and** executables
* `make format` – clang-format all sources

---

## Usage

### GUI

```bash
./render-gui path/to/model.obj
```

**Controls**

* **Left mouse drag**: orbit
* **Mouse wheel / trackpad**: dolly
* **O**: toggle orthographic / perspective
* **B**: toggle black-only mode
* **Space**: cycle vertex colors
* **C**: enter camera `(x, y, z)` directly

### CLI

```bash
# color (default)
./render-to-file input.obj <cx> <cy> <cz> perspective output.png

# black lines on white
./render-to-file input.obj <cx> <cy> <cz> perspective output.png -b

# orthographic
./render-to-file input.obj <cx> <cy> <cz> orthographic output.png
```

---

## Tests

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

---

## Code style

Add a `.clang-format` (or use the example below) and enable “Format on Save” in your IDE.

Example:

```yaml
BasedOnStyle: LLVM
IndentWidth: 4
ColumnLimit: 120
SortIncludes: false
PointerAlignment: Left
SpaceBeforeParens: ControlStatements
```

Format from the CLI:

```bash
make format
```

---

## CMake options

* `-DWERROR=ON` – treat warnings as errors
* `-DBUILD_TESTING=OFF` – skip tests
* `-DENABLE_ASAN=ON` (Debug, non-MSVC) – AddressSanitizer (if you added the option in CMake)

---

## Project layout

```
include/               # headers (math, camera, pipeline, raster, loader, app)
src/                   # renderer core
apps/
  3drendering/         # GUI (Qt widget)
  render_to_file/      # CLI (PNG save)
tests/                 # Catch2 tests (optional)
CMakeLists.txt
Makefile
README.md
report.md
```

---

## Troubleshooting

* **Qt not found** – pass its path on configure:
  macOS: `-DCMAKE_PREFIX_PATH="$(brew --prefix qt)"`
  Windows: `-DCMAKE_PREFIX_PATH="C:\Qt\6.x.x\msvcXXXX_64"`
* **“Vulkan headers” warning** – harmless; to silence, add `-DCMAKE_DISABLE_FIND_PACKAGE_WrapVulkanHeaders=ON`.
* **Executables not in repo root** – check under `build/` or `build/<config>/` if your generator ignores the runtime directory setting.
