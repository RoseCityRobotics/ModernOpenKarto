#include <gtest/gtest.h>

#include <any>
#include <string>
#include <cstdint>

// Phase 6: karto::Any (boost-derived) deleted. Verify std::any is available
// as a C++17 replacement for any code that needs type-erased storage.

TEST(AnyRemoval, StdAnyWorksAsReplacement) {
    std::any val = 42;
    EXPECT_TRUE(val.has_value());
    EXPECT_EQ(std::any_cast<int>(val), 42);

    val = std::string("hello");
    EXPECT_EQ(std::any_cast<std::string>(val), "hello");

    val.reset();
    EXPECT_FALSE(val.has_value());
}

TEST(AnyRemoval, StdAnyWithKartoTypes) {
    std::any val = double(3.14);
    EXPECT_EQ(std::any_cast<double>(val), 3.14);

    val = int32_t(-99);
    EXPECT_EQ(std::any_cast<int32_t>(val), -99);
}

TEST(AnyRemoval, StdAnyCastThrowsOnBadCast) {
    std::any val = 42;
    EXPECT_THROW(std::any_cast<std::string>(val), std::bad_any_cast);
}
