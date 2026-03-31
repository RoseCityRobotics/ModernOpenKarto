/*
 * Copyright (C) 2006-2011, SRI International (R)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <SensorData.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  SensorData::SensorData(const std::string& rSensorName)
    : m_StateId(-1)
    , m_UniqueId(-1)
    , m_SensorName(rSensorName)
    , m_Time(0)
  {
    assert(m_SensorName.size() != 0);
  }

  SensorData::~SensorData()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  LaserRangeScan::LaserRangeScan(const std::string& rSensorName)
    : SensorData(rSensorName)
  {
  }

  LaserRangeScan::LaserRangeScan(const std::string& rSensorName, const RangeReadingsList& rRangeReadings)
    : SensorData(rSensorName)
    , m_RangeReadings(rRangeReadings)
  {
  }

  LaserRangeScan::~LaserRangeScan()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  LocalizedObject::LocalizedObject(const std::string& rSensorName)
    : SensorData(rSensorName)
    , m_IsGpsReadingValid(false)
    , m_IsGpsEstimateValid(false)
  {
  }

  LocalizedObject::~LocalizedObject()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  LocalizedLaserScan::LocalizedLaserScan(const std::string& rSensorName)
    : LocalizedObject(rSensorName)
    , m_IsDirty(true)
  {
  }

  LocalizedLaserScan::~LocalizedLaserScan()
  {
  }

  void LocalizedLaserScan::Update()
  {
    ComputePointReadings();

    // set flag early, otherwise call to GetPointReadings()
    // below will recurse infinitely
    m_IsDirty = false;

    Pose2 scanPose = GetSensorPose();

    Vector2d rangePointsSum;
    const Vector2dList& rPointReadings = GetPointReadings(true);

    // calculate bounding box of scan
    m_BoundingBox = BoundingBox2();
    m_BoundingBox.Add(scanPose.GetPosition());
    for (const auto& point : rPointReadings)
    {
      m_BoundingBox.Add(point);
      rangePointsSum += point;
    }

    // compute barycenter
    double nPoints = static_cast<double>(rPointReadings.size());
    if (nPoints != 0.0)
    {
      Vector2d averagePosition = Vector2d(rangePointsSum / nPoints);
      m_BarycenterPose = Pose2(averagePosition, 0.0);
    }
    else
    {
      m_BarycenterPose = scanPose;
    }
  }

  const Vector2dList& LocalizedLaserScan::GetPointReadings(bool wantFiltered) const
  {
    if (m_IsDirty)
    {
      // throw away constness and do an update!
      const_cast<LocalizedLaserScan*>(this)->Update();
    }

    if (wantFiltered == true)
    {
      return GetFilteredPointReadings();
    }
    else
    {
      return GetUnfilteredPointReadings();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  LocalizedRangeScan::LocalizedRangeScan(const std::string& rSensorName, const std::vector<double>& rReadings)
    : LocalizedLaserScan(rSensorName)
  {
    m_RangeReadings = rReadings;
  }

  LocalizedRangeScan::~LocalizedRangeScan()
  {
  }

  // NOTE: Points are computed in world frame (heading is baked into the angle).
  // ComputeOffsets() in GridIndexLookup.h correctly inverse-transforms these
  // back to sensor-local coordinates before applying search angle rotations.
  // This has been verified to work correctly with both 180-degree and 360-degree lidars.
  void LocalizedRangeScan::ComputePointReadings()
  {
    LaserRangeFinder* pLaserRangeFinder = GetLaserRangeFinder();
    if (pLaserRangeFinder == nullptr)
    {
      return;
    }

    m_FilteredPointReadings.clear();
    m_UnfilteredPointReadings.clear();

    double minimumAngle = pLaserRangeFinder->GetMinimumAngle();
    double angularResolution = pLaserRangeFinder->GetAngularResolution();

    double minimumRange = pLaserRangeFinder->GetMinimumRange();
    double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();

    Pose2 scanPose = GetSensorPose();

    const std::vector<double>& rRangeReadings = GetRangeReadings();

    // compute point readings
    uint32_t numberOfReadings = pLaserRangeFinder->GetNumberOfRangeReadings();
    for (uint32_t i = 0; i < numberOfReadings; i++)
    {
      double angle = scanPose.GetHeading() + minimumAngle + i * angularResolution;
      double rangeReading = rRangeReadings[i];

      Vector2d point;
      point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
      point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));
      m_UnfilteredPointReadings.push_back(point);

      if (InRange(rangeReading, minimumRange, rangeThreshold))
      {
        m_FilteredPointReadings.push_back(point);
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  LocalizedPointScan::LocalizedPointScan(const std::string& rSensorName, const Vector2dList& rLocalPoints)
    : LocalizedLaserScan(rSensorName)
  {
    m_LocalPointReadings = rLocalPoints;

    Vector2d origin(0, 0);
    Pose2 originPose(origin, 0);
    for (const auto& point : rLocalPoints)
    {
      m_RangeReadings.push_back(origin.Distance(point));
      m_LocalAngles.push_back(originPose.AngleTo(point));
    }
  }

  LocalizedPointScan::~LocalizedPointScan()
  {
  }

  void LocalizedPointScan::ComputePointReadings()
  {
    LaserRangeFinder* pLaserRangeFinder = GetLaserRangeFinder();
    if (pLaserRangeFinder == nullptr)
    {
      return;
    }

    m_FilteredPointReadings.clear();
    m_UnfilteredPointReadings.clear();

    double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();
    Pose2 scanPose = GetSensorPose();
    Vector2d scanPosition = scanPose.GetPosition();

    // compute point readings
    for (uint32_t i = 0; i < m_LocalPointReadings.size(); i++)
    {
      RigidBodyTransform transform(Pose2(m_LocalPointReadings[i], 0));
      Pose2 pointPose = transform.TransformPose(scanPose);
      Vector2d point = pointPose.GetPosition();
      m_UnfilteredPointReadings.push_back(point);

      double range = scanPosition.Distance(point);
      if (InRange(range, pLaserRangeFinder->GetMinimumRange(), rangeThreshold))
      {
        m_FilteredPointReadings.push_back(point);
      }
    }
  }

}
