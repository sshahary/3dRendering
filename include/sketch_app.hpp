#pragma once
#include <vector>
#include <utility>

class SketchApp {
public:
    SketchApp(const std::vector<int>& positions,
              const std::vector<std::pair<int,int>>& edges,
              int W, int H);
    const std::vector<uint32_t>& render();
    void usePerspective(bool on);
    void orbit(float dx_deg, float dy_deg);
    void dolly(float factor);
    void resize(int W,int H);
private:
    int W_, H_; float dist_;
    std::vector<uint32_t> out_;
    std::vector<int> pos_; std::vector<std::pair<int,int>> edges_;
    void computeCenter();
};
