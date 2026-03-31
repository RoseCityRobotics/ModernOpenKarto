#include <gtest/gtest.h>

#include <Geometry.h>
#include <Sensor.h>
#include <SensorData.h>
#include <OpenMapper.h>

#include <algorithm>

// Phase 5: Verify geometry types work with std::vector storage
// after replacing karto::List. Also verify std::pair replaced karto::Pair.

// --- Vector2 with std::vector ---

TEST(Geometry, Vector2dListPushBackAndAccess) {
    karto::Vector2dList points;
    points.push_back(karto::Vector2d(1.0, 2.0));
    points.push_back(karto::Vector2d(3.0, 4.0));
    points.push_back(karto::Vector2d(5.0, 6.0));

    EXPECT_EQ(points.size(), 3u);
    EXPECT_DOUBLE_EQ(points[0].GetX(), 1.0);
    EXPECT_DOUBLE_EQ(points[1].GetY(), 4.0);
    EXPECT_DOUBLE_EQ(points[2].GetX(), 5.0);
}

TEST(Geometry, Vector2dListRangeBasedFor) {
    karto::Vector2dList points;
    points.push_back(karto::Vector2d(1.0, 0.0));
    points.push_back(karto::Vector2d(2.0, 0.0));
    points.push_back(karto::Vector2d(3.0, 0.0));

    double sum = 0.0;
    for (const auto& pt : points) {
        sum += pt.GetX();
    }
    EXPECT_DOUBLE_EQ(sum, 6.0);
}

TEST(Geometry, Vector2dListStdAlgorithms) {
    karto::Vector2dList points;
    points.push_back(karto::Vector2d(3.0, 0.0));
    points.push_back(karto::Vector2d(1.0, 0.0));
    points.push_back(karto::Vector2d(2.0, 0.0));

    std::sort(points.begin(), points.end(),
              [](const karto::Vector2d& a, const karto::Vector2d& b) {
                  return a.GetX() < b.GetX();
              });

    EXPECT_DOUBLE_EQ(points[0].GetX(), 1.0);
    EXPECT_DOUBLE_EQ(points[1].GetX(), 2.0);
    EXPECT_DOUBLE_EQ(points[2].GetX(), 3.0);
}

// --- Pose2 with std::vector ---

TEST(Geometry, Pose2ListOperations) {
    karto::Pose2List poses;
    poses.push_back(karto::Pose2(1.0, 2.0, 0.0));
    poses.push_back(karto::Pose2(3.0, 4.0, 1.57));

    EXPECT_EQ(poses.size(), 2u);
    EXPECT_DOUBLE_EQ(poses[0].GetX(), 1.0);
    EXPECT_DOUBLE_EQ(poses[1].GetHeading(), 1.57);

    poses.clear();
    EXPECT_TRUE(poses.empty());
}

TEST(Geometry, Pose2ListReserveAndResize) {
    karto::Pose2List poses;
    poses.reserve(100);
    EXPECT_GE(poses.capacity(), 100u);
    EXPECT_EQ(poses.size(), 0u);

    poses.resize(5, karto::Pose2(0.0, 0.0, 0.0));
    EXPECT_EQ(poses.size(), 5u);
}

// --- BoundingBox2 ---

TEST(Geometry, BoundingBox2AddPoints) {
    karto::BoundingBox2 bbox;
    bbox.Add(karto::Vector2d(1.0, 2.0));
    bbox.Add(karto::Vector2d(-3.0, 5.0));
    bbox.Add(karto::Vector2d(4.0, -1.0));

    EXPECT_DOUBLE_EQ(bbox.GetMinimum().GetX(), -3.0);
    EXPECT_DOUBLE_EQ(bbox.GetMinimum().GetY(), -1.0);
    EXPECT_DOUBLE_EQ(bbox.GetMaximum().GetX(), 4.0);
    EXPECT_DOUBLE_EQ(bbox.GetMaximum().GetY(), 5.0);
}

// --- RangeReadingsList (was List<kt_double>) ---

TEST(SensorData, RangeReadingsListIsVector) {
    karto::RangeReadingsList readings;
    readings.push_back(1.5);
    readings.push_back(2.7);
    readings.push_back(0.3);

    EXPECT_EQ(readings.size(), 3u);
    EXPECT_DOUBLE_EQ(readings[0], 1.5);

    // verify std algorithms work
    auto it = std::min_element(readings.begin(), readings.end());
    EXPECT_DOUBLE_EQ(*it, 0.3);
}

// --- IdPoseVector (was List<Pair<kt_int32s, Pose2>>) ---

TEST(OpenMapper, IdPoseVectorUsesStdPair) {
    karto::ScanSolver::IdPoseVector corrections;
    corrections.push_back(std::make_pair(kt_int32s(1), karto::Pose2(1.0, 2.0, 0.0)));
    corrections.push_back({kt_int32s(2), karto::Pose2(3.0, 4.0, 1.0)});

    EXPECT_EQ(corrections.size(), 2u);
    EXPECT_EQ(corrections[0].first, 1);
    EXPECT_DOUBLE_EQ(corrections[0].second.GetX(), 1.0);
    EXPECT_EQ(corrections[1].first, 2);
    EXPECT_DOUBLE_EQ(corrections[1].second.GetHeading(), 1.0);
}

// --- Vector2d basic operations ---

TEST(Geometry, Vector2dArithmetic) {
    karto::Vector2d a(1.0, 2.0);
    karto::Vector2d b(3.0, 4.0);

    karto::Vector2d sum = a + b;
    EXPECT_DOUBLE_EQ(sum.GetX(), 4.0);
    EXPECT_DOUBLE_EQ(sum.GetY(), 6.0);

    karto::Vector2d diff = b - a;
    EXPECT_DOUBLE_EQ(diff.GetX(), 2.0);
    EXPECT_DOUBLE_EQ(diff.GetY(), 2.0);

    EXPECT_NEAR(a.Length(), std::sqrt(5.0), 1e-10);
    EXPECT_DOUBLE_EQ(a.SquaredLength(), 5.0);
}

TEST(Geometry, Pose2Equality) {
    karto::Pose2 a(1.0, 2.0, 0.5);
    karto::Pose2 b(1.0, 2.0, 0.5);
    karto::Pose2 c(1.0, 2.0, 0.6);

    EXPECT_TRUE(a == b);
    EXPECT_TRUE(a != c);
}

TEST(Geometry, Pose2ToString) {
    karto::Pose2 p(1.5, 2.5, 0.0);
    std::string s = p.ToString();
    EXPECT_FALSE(s.empty());
    EXPECT_NE(s.find("1.5"), std::string::npos);
    EXPECT_NE(s.find("2.5"), std::string::npos);
}
