#include <catch2/catch_test_macros.hpp>
#include "sketch_app.hpp"
#include "tinymath.hpp"

static bool has_any_non_white(const std::vector<uint32_t>& img) {
    for (auto p : img) if (p != 0xFFFFFFFFu) return true;
    return false;
}

TEST_CASE("SketchApp renders a visible diagonal edge", "[render]") {
    std::vector<tmx::vec3> pos = { {-1.f,-1.f,0.f}, { 1.f, 1.f,0.f} };
    std::vector<std::pair<int,int>> edges = { {0,1} };

    SketchApp app(pos, edges, 64, 64);
    app.setUseOrtho(true);
    app.setCameraPosition(0.f, 0.f, 3.f);

    const auto& img = app.render();
    REQUIRE(img.size() == 64u*64u);
    REQUIRE(has_any_non_white(img));
}

TEST_CASE("Monochrome mode draws black pixels", "[render]") {
    std::vector<tmx::vec3> pos = { {-1.f,-1.f,0.f}, { 1.f, 1.f,0.f} };
    std::vector<std::pair<int,int>> edges = { {0,1} };

    SketchApp app(pos, edges, 64, 64);
    app.setUseOrtho(true);
    app.setCameraPosition(0.f, 0.f, 3.f);
    app.setMonochrome(true);

    const auto& img = app.render();
    bool saw_black = false;
    for (auto p : img) {
        if (p == 0xFF000000u) { saw_black = true; break; } // exact black ARGB
    }
    REQUIRE(saw_black);
}
