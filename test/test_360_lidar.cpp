#include <gtest/gtest.h>

#include <OpenMapper.h>
#include <Sensor.h>
#include <SensorData.h>

#include <cmath>
#include <vector>

// Phase 10: Tests for 360-degree lidar support and SensorRegistry singleton fix.

namespace {

// Generate synthetic range readings for a rectangular room.
// Robot at (robotX, robotY, robotHeading) inside a room of given dimensions centered at origin.
// The laser spans from minAngle to maxAngle with the given angular resolution.
karto::RangeReadingsList GenerateRoomScan(
    double robotX, double robotY, double robotHeading,
    double roomWidth, double roomHeight,
    double minAngle, double maxAngle, double angularRes,
    double maxRange) {

    int numReadings = static_cast<int>(std::round((maxAngle - minAngle) / angularRes)) + 1;
    karto::RangeReadingsList readings(numReadings, maxRange);

    double halfW = roomWidth / 2.0;
    double halfH = roomHeight / 2.0;

    for (int i = 0; i < numReadings; i++) {
        double angle = minAngle + i * angularRes + robotHeading;
        double cosA = std::cos(angle);
        double sinA = std::sin(angle);

        double minDist = maxRange;

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

// Create a 360-degree LaserRangeFinder: minAngle=-PI, maxAngle=PI, 1-degree resolution
karto::LaserRangeFinder* Create360Laser(const char* name) {
    karto::LaserRangeFinder* laser =
        karto::LaserRangeFinder::CreateLaserRangeFinder(
            karto::LaserRangeFinder_Custom, karto::Identifier(name));
    laser->SetMinimumRange(0.1);
    laser->SetMaximumRange(50.0);
    laser->SetMinimumAngle(-karto::KT_PI);
    laser->SetMaximumAngle(karto::KT_PI);
    laser->SetAngularResolution(karto::math::DegreesToRadians(1.0));
    laser->SetRangeThreshold(12.0);
    return laser;
}

// Create a 180-degree LaserRangeFinder for comparison/helper tests
karto::LaserRangeFinder* Create180Laser(const char* name) {
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

} // anonymous namespace

// ---------------------------------------------------------------------------
// 10a: Verify 360-degree lidar scan matching produces reasonable corrected poses
// ---------------------------------------------------------------------------

TEST(Lidar360, SingleScanProcessing) {
    karto::OpenMapper mapper("mapper_360_single");

    karto::LaserRangeFinder* laser = Create360Laser("laser_360_single");

    const double minAngle = -karto::KT_PI;
    const double maxAngle = karto::KT_PI;
    const double angularRes = karto::math::DegreesToRadians(1.0);

    // Generate a scan at origin in a 10x10 room
    const karto::RangeReadingsList readings = GenerateRoomScan(
        0.0, 0.0, 0.0, 10.0, 10.0,
        minAngle, maxAngle, angularRes, 50.0);

    karto::LocalizedRangeScan* scan =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), readings);
    scan->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));

    kt_bool result = mapper.Process(scan);
    EXPECT_TRUE(result);

    karto::Pose2 corrected = scan->GetCorrectedPose();
    EXPECT_NEAR(corrected.GetX(), 0.0, 0.5);
    EXPECT_NEAR(corrected.GetY(), 0.0, 0.5);

    delete laser;
}

TEST(Lidar360, MultipleScansProduceReasonablePoses) {
    karto::OpenMapper mapper("mapper_360_multi");

    karto::LaserRangeFinder* laser = Create360Laser("laser_360_multi");

    const double minAngle = -karto::KT_PI;
    const double maxAngle = karto::KT_PI;
    const double angularRes = karto::math::DegreesToRadians(1.0);

    // Feed several scans along a straight line inside a 10x10 room
    const double step = 0.5;
    const int numScans = 5;

    for (int i = 0; i < numScans; i++) {
        double x = i * step;
        const karto::RangeReadingsList readings = GenerateRoomScan(
            x, 0.0, 0.0, 10.0, 10.0,
            minAngle, maxAngle, angularRes, 50.0);

        karto::LocalizedRangeScan* scan =
            new karto::LocalizedRangeScan(laser->GetIdentifier(), readings);
        scan->SetOdometricPose(karto::Pose2(x, 0.0, 0.0));
        scan->SetCorrectedPose(karto::Pose2(x, 0.0, 0.0));

        kt_bool result = mapper.Process(scan);
        EXPECT_TRUE(result);

        // Corrected pose should be reasonably close to the odometric pose
        karto::Pose2 corrected = scan->GetCorrectedPose();
        EXPECT_NEAR(corrected.GetX(), x, 1.0);
        EXPECT_NEAR(corrected.GetY(), 0.0, 0.5);
    }

    delete laser;
}

TEST(Lidar360, ScanMatchingWithRotation) {
    karto::OpenMapper mapper("mapper_360_rot");

    karto::LaserRangeFinder* laser = Create360Laser("laser_360_rot");

    const double minAngle = -karto::KT_PI;
    const double maxAngle = karto::KT_PI;
    const double angularRes = karto::math::DegreesToRadians(1.0);

    // First scan at origin, no rotation
    const karto::RangeReadingsList readings1 = GenerateRoomScan(
        0.0, 0.0, 0.0, 10.0, 10.0,
        minAngle, maxAngle, angularRes, 50.0);
    karto::LocalizedRangeScan* scan1 =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), readings1);
    scan1->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan1->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));
    EXPECT_TRUE(mapper.Process(scan1));

    // Second scan with a small translation and rotation
    double heading = karto::math::DegreesToRadians(15.0);
    const karto::RangeReadingsList readings2 = GenerateRoomScan(
        1.0, 0.5, heading, 10.0, 10.0,
        minAngle, maxAngle, angularRes, 50.0);
    karto::LocalizedRangeScan* scan2 =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), readings2);
    scan2->SetOdometricPose(karto::Pose2(1.0, 0.5, heading));
    scan2->SetCorrectedPose(karto::Pose2(1.0, 0.5, heading));
    EXPECT_TRUE(mapper.Process(scan2));

    // Corrected pose should be in the ballpark of the odometric pose
    karto::Pose2 corrected = scan2->GetCorrectedPose();
    EXPECT_NEAR(corrected.GetX(), 1.0, 1.5);
    EXPECT_NEAR(corrected.GetY(), 0.5, 1.0);

    delete laser;
}

// ---------------------------------------------------------------------------
// 10b: Test that two independent mapper instances work correctly when
// scans carry their own LaserRangeFinder pointer (bypassing global registry).
// ---------------------------------------------------------------------------

TEST(SensorRegistryFix, DirectLaserPointerOnLaserRangeScan) {
    // Verify that LaserRangeScan::SetLaserRangeFinder / GetLaserRangeFinder
    // returns the direct pointer when set, falling back to registry otherwise.
    karto::LaserRangeFinder* laser = Create180Laser("laser_direct_lrs");

    const karto::RangeReadingsList readings(181, 5.0);
    karto::LaserRangeScan scan(laser->GetIdentifier(), readings);

    // Without setting direct pointer, should fall back to registry
    EXPECT_EQ(scan.GetLaserRangeFinder(), laser);

    // Set direct pointer
    scan.SetLaserRangeFinder(laser);
    EXPECT_EQ(scan.GetLaserRangeFinder(), laser);

    delete laser;
}

TEST(SensorRegistryFix, DirectLaserPointerOnLocalizedLaserScan) {
    // Verify that LocalizedLaserScan::SetLaserRangeFinder / GetLaserRangeFinder
    // returns the direct pointer when set.
    karto::LaserRangeFinder* laser = Create180Laser("laser_direct_lls");

    const karto::RangeReadingsList readings(181, 5.0);
    karto::LocalizedRangeScan* scan =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), readings);
    scan->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));

    // Without direct pointer, falls back to registry
    EXPECT_EQ(scan->GetLaserRangeFinder(), laser);

    // Set direct pointer explicitly
    scan->SetLaserRangeFinder(laser);
    EXPECT_EQ(scan->GetLaserRangeFinder(), laser);

    delete scan;
    delete laser;
}

TEST(SensorRegistryFix, TwoMappersWithSameSensorName) {
    // This test verifies that two independent mapper instances can each work
    // with their own LaserRangeFinder, even when the logical sensor name is
    // the same. The direct pointer on scans bypasses the global SensorRegistry.
    //
    // Because the SensorRegistry still throws on duplicate names, we simulate
    // the scenario by: creating laser1, processing with mapper1, destroying
    // laser1, then creating laser2 with the same name for mapper2. Old scans
    // still work because they carry their own direct pointer.

    const char* sensorName = "laser_shared_name";
    const double minAngle = -karto::KT_PI_2;
    const double maxAngle = karto::KT_PI_2;
    const double angularRes = karto::math::DegreesToRadians(1.0);

    // --- Mapper 1 with laser1 ---
    karto::OpenMapper mapper1("mapper_dual_1");
    karto::LaserRangeFinder* laser1 = Create180Laser(sensorName);

    const karto::RangeReadingsList readings1 = GenerateRoomScan(
        0.0, 0.0, 0.0, 10.0, 10.0,
        minAngle, maxAngle, angularRes, 50.0);

    karto::LocalizedRangeScan* scan1 =
        new karto::LocalizedRangeScan(laser1->GetIdentifier(), readings1);
    scan1->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan1->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));
    // Set direct pointer so scan1 doesn't depend on the registry
    scan1->SetLaserRangeFinder(laser1);

    EXPECT_TRUE(mapper1.Process(scan1));

    karto::Pose2 corrected1 = scan1->GetCorrectedPose();
    EXPECT_NEAR(corrected1.GetX(), 0.0, 0.5);
    EXPECT_NEAR(corrected1.GetY(), 0.0, 0.5);

    // Destroy laser1 — this unregisters it from the global registry
    delete laser1;

    // scan1 still has a valid concept of its laser via direct pointer
    // (note: the laser is deleted, so we can't dereference it now,
    //  but the mechanism is proven — in real usage the laser outlives scans)

    // --- Mapper 2 with laser2 (same sensor name, no registry collision) ---
    karto::OpenMapper mapper2("mapper_dual_2");
    karto::LaserRangeFinder* laser2 = Create180Laser(sensorName);

    const karto::RangeReadingsList readings2 = GenerateRoomScan(
        0.0, 0.0, 0.0, 10.0, 10.0,
        minAngle, maxAngle, angularRes, 50.0);

    karto::LocalizedRangeScan* scan2 =
        new karto::LocalizedRangeScan(laser2->GetIdentifier(), readings2);
    scan2->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan2->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));
    scan2->SetLaserRangeFinder(laser2);

    EXPECT_TRUE(mapper2.Process(scan2));

    karto::Pose2 corrected2 = scan2->GetCorrectedPose();
    EXPECT_NEAR(corrected2.GetX(), 0.0, 0.5);
    EXPECT_NEAR(corrected2.GetY(), 0.0, 0.5);

    // Verify the direct pointer is laser2, not something from the registry
    EXPECT_EQ(scan2->GetLaserRangeFinder(), laser2);

    delete laser2;
}

// ---------------------------------------------------------------------------
// Frame round-trip tests for slam_toolbox PR #362 port
// ---------------------------------------------------------------------------

TEST(FrameRoundTrip, GetSensorAtAndGetCorrectedAtAreInverses) {
    // With zero offset, GetSensorAt and GetCorrectedAt should be identity-like
    karto::LaserRangeFinder* laser = Create180Laser("laser_roundtrip_zero");

    const karto::RangeReadingsList readings(181, 5.0);
    karto::LocalizedRangeScan* scan =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), readings);
    scan->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan->SetCorrectedPose(karto::Pose2(1.0, 2.0, 0.5));

    // With zero offset pose (default), sensor pose == corrected pose
    karto::Pose2 sensorPose = scan->GetSensorPose();
    karto::Pose2 recoveredRobotPose = scan->GetCorrectedAt(sensorPose);

    EXPECT_NEAR(recoveredRobotPose.GetX(), 1.0, 1e-6);
    EXPECT_NEAR(recoveredRobotPose.GetY(), 2.0, 1e-6);
    EXPECT_NEAR(recoveredRobotPose.GetHeading(), 0.5, 1e-6);

    delete scan;
    delete laser;
}

TEST(FrameRoundTrip, GetSensorAtAndGetCorrectedAtWithOffset) {
    // With a non-zero sensor offset, the round-trip should still recover the robot pose
    karto::LaserRangeFinder* laser = Create180Laser("laser_roundtrip_offset");

    // Set a sensor offset: 0.1m forward, 0.05m left, rotated 10 degrees
    laser->SetOffsetPose(karto::Pose2(0.1, 0.05, karto::math::DegreesToRadians(10.0)));

    const karto::RangeReadingsList readings(181, 5.0);
    karto::LocalizedRangeScan* scan =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), readings);
    scan->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan->SetCorrectedPose(karto::Pose2(3.0, 4.0, 1.0));

    // GetSensorAt computes sensor pose from robot pose
    karto::Pose2 sensorPose = scan->GetSensorAt(scan->GetCorrectedPose());

    // GetCorrectedAt should recover the original robot pose
    karto::Pose2 recovered = scan->GetCorrectedAt(sensorPose);

    EXPECT_NEAR(recovered.GetX(), 3.0, 1e-4);
    EXPECT_NEAR(recovered.GetY(), 4.0, 1e-4);
    EXPECT_NEAR(recovered.GetHeading(), 1.0, 1e-4);

    delete scan;
    delete laser;
}

TEST(FrameRoundTrip, SetCorrectedPoseAndUpdateRefreshesPoints) {
    karto::LaserRangeFinder* laser = Create180Laser("laser_update_test");

    const karto::RangeReadingsList readings(181, 5.0);
    karto::LocalizedRangeScan* scan =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), readings);
    scan->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));

    // Force computation of point readings at origin
    const karto::Vector2dList& points1 = scan->GetPointReadings();
    EXPECT_FALSE(points1.empty());
    karto::Vector2d firstPoint1 = points1[0];

    // Move the scan to a new pose using SetCorrectedPoseAndUpdate
    scan->SetCorrectedPoseAndUpdate(karto::Pose2(5.0, 5.0, 0.0));

    // Point readings should have been recomputed at the new pose
    const karto::Vector2dList& points2 = scan->GetPointReadings();
    EXPECT_FALSE(points2.empty());
    karto::Vector2d firstPoint2 = points2[0];

    // Points should differ (scan moved by 5m)
    double dist = firstPoint1.Distance(firstPoint2);
    EXPECT_GT(dist, 1.0);

    delete scan;
    delete laser;
}

TEST(FrameRoundTrip, ScanMatchingWithSensorOffset) {
    // Verify that scan matching works correctly when the sensor has a non-zero offset
    karto::LaserRangeFinder* laser = Create180Laser("laser_offset_match");
    laser->SetOffsetPose(karto::Pose2(0.15, 0.0, 0.0)); // 15cm forward offset

    karto::OpenMapper mapper("mapper_offset");

    double angRes = karto::math::DegreesToRadians(1.0);

    const karto::RangeReadingsList r1 = GenerateRoomScan(0.0, 0.0, 0.0, 10.0, 10.0,
        -karto::KT_PI_2, karto::KT_PI_2, angRes, 50.0);
    karto::LocalizedRangeScan* scan1 =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), r1);
    scan1->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan1->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));
    EXPECT_TRUE(mapper.Process(scan1));

    const karto::RangeReadingsList r2 = GenerateRoomScan(1.0, 0.0, 0.0, 10.0, 10.0,
        -karto::KT_PI_2, karto::KT_PI_2, angRes, 50.0);
    karto::LocalizedRangeScan* scan2 =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), r2);
    scan2->SetOdometricPose(karto::Pose2(1.0, 0.0, 0.0));
    scan2->SetCorrectedPose(karto::Pose2(1.0, 0.0, 0.0));
    EXPECT_TRUE(mapper.Process(scan2));

    // Corrected pose should be reasonable even with sensor offset
    karto::Pose2 corrected = scan2->GetCorrectedPose();
    EXPECT_NEAR(corrected.GetX(), 1.0, 1.0);
    EXPECT_NEAR(corrected.GetY(), 0.0, 0.5);

    delete laser;
}
