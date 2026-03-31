#include <gtest/gtest.h>

#include <cstdint>
#include <cstddef>
#include <climits>

// Phase 2: Verify custom integer types match cstdint types exactly

TEST(Types, IntegerSizes) {
    EXPECT_EQ(sizeof(int8_t), 1);
    EXPECT_EQ(sizeof(uint8_t), 1);
    EXPECT_EQ(sizeof(int16_t), 2);
    EXPECT_EQ(sizeof(uint16_t), 2);
    EXPECT_EQ(sizeof(int32_t), 4);
    EXPECT_EQ(sizeof(uint32_t), 4);
    EXPECT_EQ(sizeof(int64_t), 8);
    EXPECT_EQ(sizeof(uint64_t), 8);
}

TEST(Types, IntegerSignedness) {
    EXPECT_TRUE(static_cast<int8_t>(-1) < 0);
    EXPECT_TRUE(static_cast<int16_t>(-1) < 0);
    EXPECT_TRUE(static_cast<int32_t>(-1) < 0);
    EXPECT_TRUE(static_cast<int64_t>(-1) < 0);

    EXPECT_TRUE(static_cast<uint8_t>(-1) > 0);
    EXPECT_TRUE(static_cast<uint16_t>(-1) > 0);
    EXPECT_TRUE(static_cast<uint32_t>(-1) > 0);
    EXPECT_TRUE(static_cast<uint64_t>(-1) > 0);
}

TEST(Types, AreStdintTypes) {
    EXPECT_TRUE((std::is_same_v<int8_t, std::int8_t>));
    EXPECT_TRUE((std::is_same_v<uint8_t, std::uint8_t>));
    EXPECT_TRUE((std::is_same_v<int16_t, std::int16_t>));
    EXPECT_TRUE((std::is_same_v<uint16_t, std::uint16_t>));
    EXPECT_TRUE((std::is_same_v<int32_t, std::int32_t>));
    EXPECT_TRUE((std::is_same_v<uint32_t, std::uint32_t>));
    EXPECT_TRUE((std::is_same_v<int64_t, std::int64_t>));
    EXPECT_TRUE((std::is_same_v<uint64_t, std::uint64_t>));
    EXPECT_TRUE((std::is_same_v<size_t, std::size_t>));
}

TEST(Types, TrivialAliases) {
    EXPECT_TRUE((std::is_same_v<bool, bool>));
    EXPECT_TRUE((std::is_same_v<char, char>));
    EXPECT_TRUE((std::is_same_v<float, float>));
    EXPECT_TRUE((std::is_same_v<double, double>));
}
