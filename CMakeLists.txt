cmake_minimum_required(VERSION 4.0)
project(3drendering LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 COMPONENTS Widgets REQUIRED)
add_executable(3drendering
    apps/3drendering/main.cpp
    src/mesh_loader.cpp
    src/camera.cpp
    src/pipeline3d.cpp
    src/viewport.cpp
    src/line_raster.cpp
    src/projection.cpp
    src/transform.cpp
    src/sketch_app.cpp
)
target_include_directories(3drendering PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(3drendering PRIVATE Qt6::Widgets)
