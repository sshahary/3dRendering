cmake_minimum_required(VERSION 3.20)
project(3dwireframe LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# tests
include(CTest)                 # defines BUILD_TESTING ON by default

if (BUILD_TESTING)
  find_package(Catch2 3 QUIET) # try system Catch2 first
  if (NOT Catch2_FOUND)
    message(STATUS "Catch2 not found locally, fetching it")
    include(FetchContent)
    FetchContent_Declare(
      Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG v3.5.3    # any v3 tag is fine
    )
    FetchContent_MakeAvailable(Catch2)
  endif()

  add_executable(core_tests tests/core_tests.cpp)
  target_link_libraries(core_tests PRIVATE renderer_core Catch2::Catch2WithMain)
  target_include_directories(core_tests PRIVATE ${CMAKE_SOURCE_DIR}/include)

  include(Catch)
  catch_discover_tests(core_tests)
endif()

# Qt (needed: Widgets for GUI, Gui for QImage PNG in CLI)
set(CMAKE_AUTOMOC ON)
find_package(Qt6 REQUIRED COMPONENTS Widgets Gui)

# Minimal warnings (toggle -Werror via -DWERROR=ON)
option(WERROR "Treat warnings as errors" OFF)
if (MSVC)
  add_compile_options(/W4 $<$<BOOL:${WERROR}>:/WX>)
else()
  add_compile_options(-Wall -Wextra -Wpedantic $<$<BOOL:${WERROR}>:-Werror>)
endif()

# Optional: drop binaries in repo root (comment out if you prefer build/…)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
foreach(cfg IN ITEMS Debug Release RelWithDebInfo MinSizeRel)
  string(TOUPPER "${cfg}" CFGU)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CFGU} "${CMAKE_SOURCE_DIR}")
endforeach()

# ----- Core (all engine code in src/, headers in include/)
file(GLOB CORE_SOURCES CONFIGURE_DEPENDS "src/*.cpp")
add_library(renderer_core ${CORE_SOURCES})
target_include_directories(renderer_core PUBLIC ${CMAKE_SOURCE_DIR}/include)

# ----- GUI exe: apps/3drendering/*
add_executable(render-gui
  apps/3drendering/main.cpp
  apps/3drendering/controls.cpp
)
target_include_directories(render-gui PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/apps/3drendering
)
target_link_libraries(render-gui PRIVATE renderer_core Qt6::Widgets)

# ----- CLI exe: apps/render_to_file/*
add_executable(render-to-file
  apps/render_to_file/main.cpp
)
target_include_directories(render-to-file PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(render-to-file PRIVATE renderer_core Qt6::Gui)  # for QImage::save()
