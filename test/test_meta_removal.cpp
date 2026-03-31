#include <gtest/gtest.h>

#include <Meta.h>
#include <Object.h>
#include <Sensor.h>
#include <SensorData.h>
#include <Objects.h>
#include <OccupancyGrid.h>
#include <TypeCasts.h>

// Phase 3: Verify KARTO_RTTI / dynamic_cast type checks still work
// after removing the meta/reflection system.

namespace {

// Verify KARTO_TYPE macro still generates KartoTypeId specializations
TEST(MetaRemoval, KartoTypeProducesTypeNames) {
    const char* sensorName = karto::GetKartoTypeIdTemplate<karto::Sensor>();
    EXPECT_STREQ(sensorName, "Sensor");

    const char* laserName = karto::GetKartoTypeIdTemplate<karto::LaserRangeFinder>();
    EXPECT_STREQ(laserName, "LaserRangeFinder");
}

// Verify KARTO_RTTI virtual dispatch works via GetKartoClassId
TEST(MetaRemoval, KartoRttiReturnsCorrectType) {
    karto::Identifier id("test_laser");
    karto::LaserRangeFinder* laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
        karto::LaserRangeFinder_Custom, id);

    // GetKartoClassId should return the derived type via KARTO_RTTI virtual dispatch
    const char* classId = laser->GetKartoClassId();
    EXPECT_STREQ(classId, "LaserRangeFinder");

    // Verify via base pointer — virtual dispatch returns derived type
    karto::Sensor* sensorPtr = laser;
    const char* baseClassId = sensorPtr->GetKartoClassId();
    EXPECT_STREQ(baseClassId, "LaserRangeFinder");
}

// Verify KARTO_TYPECHECKCAST macros (dynamic_cast-based) still work
TEST(MetaRemoval, TypeCheckCastsWork) {
    karto::Identifier id("test_laser");
    karto::LaserRangeFinder* laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
        karto::LaserRangeFinder_Custom, id);

    karto::Object* obj = laser;

    EXPECT_TRUE(karto::IsSensor(obj));
    EXPECT_TRUE(karto::IsLaserRangeFinder(obj));
    EXPECT_FALSE(karto::IsDrive(obj));
    EXPECT_FALSE(karto::IsOccupancyGrid(obj));
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

} // anonymous namespace
