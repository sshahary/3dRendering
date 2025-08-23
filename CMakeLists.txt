cmake_minimum_required(VERSION 3.20)
project(3drendering LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt6 REQUIRED COMPONENTS Widgets Gui)

# put built executables in the repo root (optional)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
foreach(cfg IN ITEMS Debug Release RelWithDebInfo MinSizeRel)
  string(TOUPPER "${cfg}" CFGU)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CFGU} "${CMAKE_SOURCE_DIR}")
endforeach()

# --- core library (link both apps to this)
add_library(renderer_core
    src/mesh_loader.cpp
    src/camera.cpp
    src/pipeline3d.cpp
    src/viewport.cpp
    src/line_raster.cpp
    src/projection.cpp
    src/transform.cpp
    src/sketch_app.cpp
)
target_include_directories(renderer_core PUBLIC ${CMAKE_SOURCE_DIR}/include)

# --- GUI app: target name 'rendergui' -> file name 'render-gui'
add_executable(rendergui
  apps/3drendering/main.cpp
  apps/3drendering/controls.cpp
  apps/3drendering/controls.hpp
)
set_target_properties(rendergui PROPERTIES OUTPUT_NAME "render-gui")
target_include_directories(rendergui PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/apps/3drendering)
target_link_libraries(rendergui PRIVATE renderer_core Qt6::Widgets)

# --- CLI app: target name 'rendertofile' -> file name 'render-to-file'
add_executable(rendertofile
  apps/render_to_file/main.cpp
)
set_target_properties(rendertofile PROPERTIES OUTPUT_NAME "render-to-file")
target_include_directories(rendertofile PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rendertofile PRIVATE renderer_core Qt6::Gui)  # QImage PNG write
