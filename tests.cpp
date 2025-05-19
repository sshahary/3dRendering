#include "lib.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("foo") {
    CHECK(foo() == 42);
}
