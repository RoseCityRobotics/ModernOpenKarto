#include <gtest/gtest.h>

#include <OpenMapper.h>
#include <Sensor.h>
#include <SensorData.h>

#include <cmath>
#include <vector>

// Phase 8: Verify multi-threaded CorrelateScan (std::thread-based)
// produces same results as single-threaded path.

namespace {

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

karto::RangeReadingsList GenerateRoomScan(
    double robotX, double robotY, double robotHeading,
    double roomWidth, double roomHeight,
    double minAngle, double maxAngle, double angularRes) {

    int numReadings = static_cast<int>(std::round((maxAngle - minAngle) / angularRes)) + 1;
    karto::RangeReadingsList readings(numReadings, 50.0);

    double halfW = roomWidth / 2.0;
    double halfH = roomHeight / 2.0;

    for (int i = 0; i < numReadings; i++) {
        double angle = minAngle + i * angularRes + robotHeading;
        double cosA = std::cos(angle);
        double sinA = std::sin(angle);
        double minDist = 50.0;

        if (cosA > 1e-6) {
            double d = (halfW - robotX) / cosA;
            double wy = robotY + d * sinA;
            if (d > 0 && std::abs(wy) <= halfH && d < minDist) minDist = d;
        }
        if (cosA < -1e-6) {
            double d = (-halfW - robotX) / cosA;
            double wy = robotY + d * sinA;
            if (d > 0 && std::abs(wy) <= halfH && d < minDist) minDist = d;
        }
        if (sinA > 1e-6) {
            double d = (halfH - robotY) / sinA;
            double wx = robotX + d * cosA;
            if (d > 0 && std::abs(wx) <= halfW && d < minDist) minDist = d;
        }
        if (sinA < -1e-6) {
            double d = (-halfH - robotY) / sinA;
            double wx = robotX + d * cosA;
            if (d > 0 && std::abs(wx) <= halfW && d < minDist) minDist = d;
        }

        readings[i] = minDist;
    }
    return readings;
}

// Run mapper and return corrected pose of second scan
karto::Pose2 RunTwoScans(bool multiThreaded, const char* laserName) {
    karto::OpenMapper mapper("mapper", multiThreaded);

    karto::LaserRangeFinder* laser = CreateTestLaser(laserName);

    double angRes = karto::math::DegreesToRadians(1.0);

    karto::RangeReadingsList r1 = GenerateRoomScan(0.0, 0.0, 0.0, 10.0, 10.0,
        -karto::KT_PI_2, karto::KT_PI_2, angRes);
    karto::LocalizedRangeScan* scan1 =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), r1);
    scan1->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
    scan1->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));
    mapper.Process(scan1);

    karto::RangeReadingsList r2 = GenerateRoomScan(0.5, 0.0, 0.0, 10.0, 10.0,
        -karto::KT_PI_2, karto::KT_PI_2, angRes);
    karto::LocalizedRangeScan* scan2 =
        new karto::LocalizedRangeScan(laser->GetIdentifier(), r2);
    scan2->SetOdometricPose(karto::Pose2(0.5, 0.0, 0.0));
    scan2->SetCorrectedPose(karto::Pose2(0.5, 0.0, 0.0));
    mapper.Process(scan2);

    karto::Pose2 result = scan2->GetCorrectedPose();
    delete laser;
    return result;
}

} // anonymous namespace

TEST(ParallelScanMatch, SingleThreadedProducesReasonableResult) {
    karto::Pose2 pose = RunTwoScans(false, "laser_st");
    EXPECT_NEAR(pose.GetX(), 0.5, 1.0);
    EXPECT_NEAR(pose.GetY(), 0.0, 0.5);
}

TEST(ParallelScanMatch, MultiThreadedProducesReasonableResult) {
    karto::Pose2 pose = RunTwoScans(true, "laser_mt");
    EXPECT_NEAR(pose.GetX(), 0.5, 1.0);
    EXPECT_NEAR(pose.GetY(), 0.0, 0.5);
}

TEST(ParallelScanMatch, SingleAndMultiThreadedAgree) {
    karto::Pose2 stPose = RunTwoScans(false, "laser_agree_st");
    karto::Pose2 mtPose = RunTwoScans(true, "laser_agree_mt");

    // Both threading modes should produce the same corrected pose
    EXPECT_NEAR(stPose.GetX(), mtPose.GetX(), 0.01);
    EXPECT_NEAR(stPose.GetY(), mtPose.GetY(), 0.01);
    EXPECT_NEAR(stPose.GetHeading(), mtPose.GetHeading(), 0.01);
}
