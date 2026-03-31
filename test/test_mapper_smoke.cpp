#include <gtest/gtest.h>

#include <OpenMapper.h>
#include <Sensor.h>
#include <SensorData.h>

#include <cmath>
#include <vector>

// Phase 7: Mapper smoke test — exercises the core SLAM pipeline end-to-end.
// This is the critical regression test after replacing SmartPointer/Referenced
// with std::shared_ptr and verifying memory management still works.

namespace {

// Create a simple LaserRangeFinder with 180-degree FOV, 1-degree resolution
karto::LaserRangeFinder* CreateTestLaser(const char* name) {
    karto::LaserRangeFinder* laser =
        karto::LaserRangeFinder::CreateLaserRangeFinder(
            karto::LaserRangeFinder_Custom, karto::Identifier(name));
    laser->SetMinimumRange(0.1);
    laser->SetMaximumRange(50.0);
    laser->SetMinimumAngle(-karto::KT_PI_2);
    laser->SetMaximumAngle(karto::KT_PI_2);
    laser->SetAngularResolution(karto::math::DegreesToRadians(1.0));
    laser->SetRangeThreshold(12.0);
    return laser;
}

// Generate synthetic range readings for a rectangular room
// Robot at (x,y,heading) inside a 10m x 10m room centered at origin
karto::RangeReadingsList GenerateRoomScan(
    double robotX, double robotY, double robotHeading,
    double roomWidth, double roomHeight,
    double minAngle, double maxAngle, double angularRes) {

    int numReadings = static_cast<int>(std::round((maxAngle - minAngle) / angularRes)) + 1;
    karto::RangeReadingsList readings(numReadings, 50.0); // default to max range

    double halfW = roomWidth / 2.0;
    double halfH = roomHeight / 2.0;

    for (int i = 0; i < numReadings; i++) {
        double angle = minAngle + i * angularRes + robotHeading;
        double cosA = std::cos(angle);
        double sinA = std::sin(angle);

        double minDist = 50.0;

        // Check intersection with 4 walls
        // Right wall: x = halfW
        if (cosA > 1e-6) {
            double d = (halfW - robotX) / cosA;
            double wy = robotY + d * sinA;
            if (d > 0 && std::abs(wy) <= halfH && d < minDist) minDist = d;
        }
        // Left wall: x = -halfW
        if (cosA < -1e-6) {
            double d = (-halfW - robotX) / cosA;
            double wy = robotY + d * sinA;
            if (d > 0 && std::abs(wy) <= halfH && d < minDist) minDist = d;
        }
        // Top wall: y = halfH
        if (sinA > 1e-6) {
            double d = (halfH - robotY) / sinA;
            double wx = robotX + d * cosA;
            if (d > 0 && std::abs(wx) <= halfW && d < minDist) minDist = d;
        }
        // Bottom wall: y = -halfH
        if (sinA < -1e-6) {
            double d = (-halfH - robotY) / sinA;
            double wx = robotX + d * cosA;
            if (d > 0 && std::abs(wx) <= halfW && d < minDist) minDist = d;
        }

        readings[i] = minDist;
    }
    return readings;
}

} // anonymous namespace

TEST(MapperSmoke, CreateAndDestroyMapper) {
    karto::OpenMapper mapper("test_mapper");
    // Verify mapper exists and can be configured
    EXPECT_FALSE(mapper.GetIdentifier().GetName().empty());
}

TEST(MapperSmoke, ProcessSingleScan) {
    karto::OpenMapper mapper("test_mapper");

    karto::LaserRangeFinder* laser = CreateTestLaser("laser_single");

    // Generate a scan at origin facing forward
    const karto::RangeReadingsList readings = GenerateRoomScan(
        0.0, 0.0, 0.0, 10.0, 10.0,
        -karto::KT_PI_2, karto::KT_PI_2, karto::math::DegreesToRadians(1.0));

    karto::LocalizedRangeScan* scan =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), readings);
    scan->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));

    // First scan should always succeed
    kt_bool result = mapper.Process(scan);
    EXPECT_TRUE(result);

    // Corrected pose should be near origin
    karto::Pose2 corrected = scan->GetCorrectedPose();
    EXPECT_NEAR(corrected.GetX(), 0.0, 0.5);
    EXPECT_NEAR(corrected.GetY(), 0.0, 0.5);

    delete laser;
}

TEST(MapperSmoke, ProcessMultipleScans) {
    karto::OpenMapper mapper("test_mapper");

    karto::LaserRangeFinder* laser = CreateTestLaser("laser_multi");

    // Feed several scans along a straight line
    double step = 0.5; // 50cm steps
    int numScans = 5;

    for (int i = 0; i < numScans; i++) {
        double x = i * step;
        const karto::RangeReadingsList readings = GenerateRoomScan(
            x, 0.0, 0.0, 10.0, 10.0,
            -karto::KT_PI_2, karto::KT_PI_2, karto::math::DegreesToRadians(1.0));

        karto::LocalizedRangeScan* scan =
            new karto::LocalizedRangeScan(laser->GetIdentifier(), readings);
        scan->SetOdometricPose(karto::Pose2(x, 0.0, 0.0));
        scan->SetCorrectedPose(karto::Pose2(x, 0.0, 0.0));

        kt_bool result = mapper.Process(scan);
        EXPECT_TRUE(result);
    }

    delete laser;
}

TEST(MapperSmoke, CorrectedPosesAreReasonable) {
    karto::OpenMapper mapper("test_mapper");

    karto::LaserRangeFinder* laser = CreateTestLaser("laser_corrected");

    // Feed two scans with a known displacement
    const karto::RangeReadingsList readings1 = GenerateRoomScan(
        0.0, 0.0, 0.0, 10.0, 10.0,
        -karto::KT_PI_2, karto::KT_PI_2, karto::math::DegreesToRadians(1.0));
    karto::LocalizedRangeScan* scan1 =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), readings1);
    scan1->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan1->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));
    mapper.Process(scan1);

    const karto::RangeReadingsList readings2 = GenerateRoomScan(
        1.0, 0.0, 0.0, 10.0, 10.0,
        -karto::KT_PI_2, karto::KT_PI_2, karto::math::DegreesToRadians(1.0));
    karto::LocalizedRangeScan* scan2 =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), readings2);
    scan2->SetOdometricPose(karto::Pose2(1.0, 0.0, 0.0));
    scan2->SetCorrectedPose(karto::Pose2(1.0, 0.0, 0.0));
    mapper.Process(scan2);

    // The corrected pose should be near the odometric pose
    // (scan matching in a symmetric room with known geometry)
    karto::Pose2 corrected = scan2->GetCorrectedPose();
    EXPECT_NEAR(corrected.GetX(), 1.0, 1.0);  // within 1m
    EXPECT_NEAR(corrected.GetY(), 0.0, 0.5);

    delete laser;
}
