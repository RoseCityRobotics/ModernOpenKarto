#include <gtest/gtest.h>

#include <Types.h>
#include <cstdint>
#include <climits>

// Phase 2: Verify custom integer types match cstdint types exactly

TEST(Types, IntegerSizes) {
    EXPECT_EQ(sizeof(kt_int8s), 1);
    EXPECT_EQ(sizeof(kt_int8u), 1);
    EXPECT_EQ(sizeof(kt_int16s), 2);
    EXPECT_EQ(sizeof(kt_int16u), 2);
    EXPECT_EQ(sizeof(kt_int32s), 4);
    EXPECT_EQ(sizeof(kt_int32u), 4);
    EXPECT_EQ(sizeof(kt_int64s), 8);
    EXPECT_EQ(sizeof(kt_int64u), 8);
}

TEST(Types, IntegerSignedness) {
    EXPECT_TRUE(static_cast<kt_int8s>(-1) < 0);
    EXPECT_TRUE(static_cast<kt_int16s>(-1) < 0);
    EXPECT_TRUE(static_cast<kt_int32s>(-1) < 0);
    EXPECT_TRUE(static_cast<kt_int64s>(-1) < 0);

    EXPECT_TRUE(static_cast<kt_int8u>(-1) > 0);
    EXPECT_TRUE(static_cast<kt_int16u>(-1) > 0);
    EXPECT_TRUE(static_cast<kt_int32u>(-1) > 0);
    EXPECT_TRUE(static_cast<kt_int64u>(-1) > 0);
}

TEST(Types, AreStdintTypes) {
    EXPECT_TRUE((std::is_same_v<kt_int8s, std::int8_t>));
    EXPECT_TRUE((std::is_same_v<kt_int8u, std::uint8_t>));
    EXPECT_TRUE((std::is_same_v<kt_int16s, std::int16_t>));
    EXPECT_TRUE((std::is_same_v<kt_int16u, std::uint16_t>));
    EXPECT_TRUE((std::is_same_v<kt_int32s, std::int32_t>));
    EXPECT_TRUE((std::is_same_v<kt_int32u, std::uint32_t>));
    EXPECT_TRUE((std::is_same_v<kt_int64s, std::int64_t>));
    EXPECT_TRUE((std::is_same_v<kt_int64u, std::uint64_t>));
    EXPECT_TRUE((std::is_same_v<kt_size_t, std::size_t>));
}

TEST(Types, TrivialAliases) {
    EXPECT_TRUE((std::is_same_v<kt_bool, bool>));
    EXPECT_TRUE((std::is_same_v<kt_char, char>));
    EXPECT_TRUE((std::is_same_v<kt_float, float>));
    EXPECT_TRUE((std::is_same_v<kt_double, double>));
}
