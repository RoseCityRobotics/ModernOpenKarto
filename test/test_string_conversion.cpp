#include <gtest/gtest.h>

#include <Geometry.h>
#include <Sensor.h>
#include <OpenMapper.h>

// Phase 4-7: Verify std::string works throughout karto::ToString / karto::FromString.
// Object/Identifier/Parameter systems have been removed.

// --- karto::ToString ---

TEST(ToString, ToStringBool) {
    EXPECT_EQ(karto::ToString(true), "true");
    EXPECT_EQ(karto::ToString(false), "false");
}

TEST(ToString, ToStringIntegers) {
    EXPECT_EQ(karto::ToString(int32_t(0)), "0");
    EXPECT_EQ(karto::ToString(int32_t(-42)), "-42");
    EXPECT_EQ(karto::ToString(uint32_t(12345)), "12345");
    EXPECT_EQ(karto::ToString(int64_t(-999999999LL)), "-999999999");
}

TEST(ToString, ToStringDouble) {
    std::string result = karto::ToString(3.14159, 4);
    EXPECT_EQ(result, "3.1416");
}

TEST(ToString, ToStringSizeT) {
    size_t val = 42;
    std::string result = karto::ToString(val);
    EXPECT_EQ(result, "42");
}

// --- karto::FromString ---

TEST(FromString, FromStringBool) {
    bool value = false;
    EXPECT_TRUE(karto::FromString("true", value));
    EXPECT_TRUE(value);

    EXPECT_TRUE(karto::FromString("false", value));
    EXPECT_FALSE(value);
}

TEST(FromString, FromStringInt32) {
    int32_t value = 0;
    EXPECT_TRUE(karto::FromString("-42", value));
    EXPECT_EQ(value, -42);
}

TEST(FromString, FromStringDouble) {
    double value = 0.0;
    EXPECT_TRUE(karto::FromString("3.14159", value));
    EXPECT_NEAR(value, 3.14159, 1e-5);
}

// --- std::ostringstream (replaces StringBuilder) ---

TEST(OStringStream, AppendAndStr) {
    std::ostringstream ss;
    ss << "hello" << " " << int32_t(42) << " " << 3.14;
    std::string result = ss.str();
    EXPECT_NE(result.find("hello"), std::string::npos);
    EXPECT_NE(result.find("42"), std::string::npos);
}

TEST(OStringStream, AppendSizeT) {
    std::ostringstream ss;
    size_t val = 99;
    ss << "count=" << val;
    std::string result = ss.str();
    EXPECT_NE(result.find("count=99"), std::string::npos);
}

// --- Sensor name (replaces Identifier tests) ---

TEST(SensorName, LaserRangeFinderHasName) {
    karto::LaserRangeFinder* laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
        karto::LaserRangeFinder_Custom, "TestSensor");
    EXPECT_EQ(laser->GetName(), "TestSensor");
    delete laser;
}

TEST(SensorName, LaserRangeFinderDefaultName) {
    karto::LaserRangeFinder* laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
        karto::LaserRangeFinder_Custom, "");
    EXPECT_EQ(laser->GetName(), "User-Defined LaserRangeFinder");
    delete laser;
}

// --- MapperConfig (replaces Parameter tests) ---

TEST(MapperConfig, DefaultValues) {
    karto::MapperConfig config;
    EXPECT_TRUE(config.useScanMatching);
    EXPECT_TRUE(config.useScanBarycenter);
    EXPECT_NEAR(config.minimumTravelDistance, 0.2, 1e-10);
    EXPECT_EQ(config.scanBufferSize, 70u);
    EXPECT_NEAR(config.correlationSearchSpaceDimension, 0.3, 1e-10);
    EXPECT_NEAR(config.loopMatchMinimumResponseFine, 0.7, 1e-10);
}

TEST(MapperConfig, ModifyValues) {
    karto::MapperConfig config;
    config.useScanMatching = false;
    EXPECT_FALSE(config.useScanMatching);

    config.minimumTravelDistance = 0.5;
    EXPECT_NEAR(config.minimumTravelDistance, 0.5, 1e-10);
}

TEST(MapperConfig, MapperUsesConfig) {
    karto::OpenMapper mapper("config_test");
    mapper.GetConfig().useScanMatching = false;
    EXPECT_FALSE(mapper.GetConfig().useScanMatching);
}
