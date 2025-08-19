cmake_minimum_required(VERSION 4.0)

find_package(Qt6 COMPONENTS Widgets REQUIRED)
add_executable(3drendering
    apps/3drendering/main.cpp
    src/mesh_loader.cpp
    src/sketch_app.cpp
)
target_include_directories(3drendering PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(3drendering PRIVATE Qt6::Widgets)
