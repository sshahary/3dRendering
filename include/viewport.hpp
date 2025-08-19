#pragma once
#include <vector>
#include "tinymath.hpp"

class Viewport {
public:
    Viewport(int w,int h);
    void resize(int w,int h);
    std::vector<tmx::ivec2> toScreen(const std::vector<tmx::vec4>& ndc) const;
private:
    int w_, h_;
};
