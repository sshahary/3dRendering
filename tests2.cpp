#include "lib.hpp"

#include <gtest/gtest.h>

TEST(Foo, Is42) {
    ASSERT_EQ(foo(), 42);
}
