cmake_minimum_required(VERSION 3.20)
project(3dwireframe LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_ASAN "Enable AddressSanitizer (Debug only, non-MSVC)" OFF)

# Enable ASAN globally (simple + minimal)
if (ENABLE_ASAN AND CMAKE_BUILD_TYPE STREQUAL "Debug" AND NOT MSVC)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
  message(STATUS "AddressSanitizer enabled")
endif()

# Put executables in the repo root
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
foreach(cfg IN ITEMS Debug Release RelWithDebInfo MinSizeRel)
  string(TOUPPER "${cfg}" CFGU)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CFGU} "${CMAKE_SOURCE_DIR}")
endforeach()

# Qt 6 (Widgets for GUI, Gui for QImage in CLI)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt6 QUIET COMPONENTS Widgets Gui)
if(NOT Qt6_FOUND)
  message(FATAL_ERROR
    "Qt6 not found.\n"
    "Hints:\n"
    "  macOS Homebrew:   cmake -S . -B build -DCMAKE_PREFIX_PATH=\"$(brew --prefix qt)\"\n"
    "  Ubuntu Debian:    sudo apt install qt6-base-dev\n"
    "  Windows MSVC:     set CMAKE_PREFIX_PATH=C:/Qt/6.x.x/msvcXXXX_64")
endif()

# Minimal warnings, optional Werror
option(WERROR "Treat warnings as errors" OFF)
if(MSVC)
  add_compile_options(/W4 $<$<BOOL:${WERROR}>:/WX>)
else()
  add_compile_options(-Wall -Wextra -Wpedantic $<$<BOOL:${WERROR}>:-Werror>)
endif()

# Core library
file(GLOB CORE_SOURCES CONFIGURE_DEPENDS "src/*.cpp")
add_library(renderer_core ${CORE_SOURCES})
target_include_directories(renderer_core PUBLIC ${CMAKE_SOURCE_DIR}/include)

# GUI executable
add_executable(render-gui
  apps/3drendering/main.cpp
  apps/3drendering/controls.cpp
)
target_include_directories(render-gui PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/apps/3drendering
)
target_link_libraries(render-gui PRIVATE renderer_core Qt6::Widgets)

# CLI executable
add_executable(render-to-file
  apps/render_to_file/main.cpp
)
target_include_directories(render-to-file PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(render-to-file PRIVATE renderer_core Qt6::Gui)

# Tests (optional, auto fetch Catch2 if missing)
include(CTest)
if(BUILD_TESTING)
  find_package(Catch2 3 QUIET)
  if(NOT Catch2_FOUND)
    include(FetchContent)
    FetchContent_Declare(
      Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG v3.5.3
    )
    FetchContent_MakeAvailable(Catch2)
  endif()
  add_executable(core_tests tests/core_tests.cpp)
  target_link_libraries(core_tests PRIVATE renderer_core Catch2::Catch2WithMain)
  target_include_directories(core_tests PRIVATE ${CMAKE_SOURCE_DIR}/include)
  include(Catch)
  catch_discover_tests(core_tests)
endif()
