cmake_minimum_required(VERSION 3.20)

project(3drendering LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt meta object tools
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Put executables in the source root instead of build/
# For multi config generators also set per config
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
foreach(cfg IN ITEMS Debug Release RelWithDebInfo MinSizeRel)
  string(TOUPPER "${cfg}" CFGU)
  set("CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CFGU}" "${CMAKE_SOURCE_DIR}")
endforeach()

# Warnings and optional ASAN
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
elseif(MSVC)
  add_compile_options(/W4)
endif()

option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
if(ENABLE_ASAN AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address)
endif()

find_package(Qt6 COMPONENTS Widgets REQUIRED)

add_executable(3drendering
    apps/3drendering/main.cpp
    apps/3drendering/controls.cpp
    apps/3drendering/controls.hpp            
    src/mesh_loader.cpp
    src/camera.cpp
    src/pipeline3d.cpp
    src/viewport.cpp
    src/line_raster.cpp
    src/projection.cpp
    src/transform.cpp
    src/sketch_app.cpp
)
target_include_directories(3drendering PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/3drendering   # for controls.hpp
)
target_link_libraries(3drendering PRIVATE Qt6::Widgets)
