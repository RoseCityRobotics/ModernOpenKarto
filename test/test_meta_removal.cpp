#include <gtest/gtest.h>

#include <Sensor.h>
#include <SensorData.h>
#include <OccupancyGrid.h>

// Phase F: Verify dynamic_cast type checks still work
// after removing Object base class, Identifier, and Parameter systems.

namespace {

// Verify dynamic_cast type checks work on Sensor hierarchy
TEST(MetaRemoval, TypeCheckCastsWork) {
    karto::LaserRangeFinder* laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
        karto::LaserRangeFinder_Custom, "test_laser_casts");

    karto::Sensor* sensor = laser;

    EXPECT_TRUE(dynamic_cast<karto::Sensor*>(laser) != nullptr);
    EXPECT_TRUE(dynamic_cast<karto::LaserRangeFinder*>(sensor) != nullptr);
    EXPECT_TRUE(dynamic_cast<karto::Drive*>(sensor) == nullptr);

    delete laser;
}

// Verify LaserRangeFinderType enum still works after removing MetaEnum registration
TEST(MetaRemoval, LaserRangeFinderTypeEnumExists) {
    EXPECT_EQ(karto::LaserRangeFinder_Custom, 0);
    EXPECT_EQ(karto::LaserRangeFinder_Sick_LMS100, 1);
    EXPECT_EQ(karto::LaserRangeFinder_Sick_LMS200, 2);
    EXPECT_EQ(karto::LaserRangeFinder_Sick_LMS291, 3);
    EXPECT_EQ(karto::LaserRangeFinder_Hokuyo_UTM_30LX, 4);
    EXPECT_EQ(karto::LaserRangeFinder_Hokuyo_URG_04LX, 5);
}

// Verify sensor name works after replacing Identifier with std::string
TEST(MetaRemoval, SensorNameWorks) {
    karto::LaserRangeFinder* laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
        karto::LaserRangeFinder_Custom, "my_laser");
    EXPECT_EQ(laser->GetName(), "my_laser");
    delete laser;
}

// Verify direct fields work on LaserRangeFinder after removing Parameter<T>
TEST(MetaRemoval, LaserDirectFieldsWork) {
    karto::LaserRangeFinder* laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
        karto::LaserRangeFinder_Custom, "field_test");

    laser->SetMinimumRange(0.5);
    EXPECT_NEAR(laser->GetMinimumRange(), 0.5, 1e-10);

    laser->SetMaximumRange(30.0);
    EXPECT_NEAR(laser->GetMaximumRange(), 30.0, 1e-10);

    laser->SetRangeThreshold(12.0);
    EXPECT_NEAR(laser->GetRangeThreshold(), 12.0, 1e-10);

    EXPECT_EQ(laser->GetType(), karto::LaserRangeFinder_Custom);

    delete laser;
}

} // anonymous namespace
