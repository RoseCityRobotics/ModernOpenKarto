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

  struct SensorDataPrivate
  {
    /**
     * Custom data
     */
    CustomItemList m_CustomItems;
  };

  SensorData::SensorData(const Identifier& rSensorIdentifier)
    : Object()
    , m_pSensorDataPrivate(new SensorDataPrivate())
    , m_StateId(-1)
    , m_UniqueId(-1)
    , m_SensorIdentifier(rSensorIdentifier)
    , m_Time(0)
  {
    assert(m_SensorIdentifier.Size() != 0);
  }

  SensorData::~SensorData()
  {
    m_pSensorDataPrivate->m_CustomItems.clear();

    delete m_pSensorDataPrivate;    
  }

  void SensorData::AddCustomItem(CustomItem* pCustomData)
  {
    m_pSensorDataPrivate->m_CustomItems.push_back(pCustomData);
  }

  const CustomItemList& SensorData::GetCustomItems() const
  {
    return m_pSensorDataPrivate->m_CustomItems;
  }

  kt_bool SensorData::HasCustomItem() 
  {
    return m_pSensorDataPrivate->m_CustomItems.size() > 0;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  LaserRangeScan::LaserRangeScan(const Identifier& rSensorIdentifier)
    : SensorData(rSensorIdentifier)
  {
  }

  LaserRangeScan::LaserRangeScan(const Identifier& rSensorIdentifier, const RangeReadingsList& rRangeReadings)
    : SensorData(rSensorIdentifier)
    , m_RangeReadings(rRangeReadings)
  {
  }

  LaserRangeScan::~LaserRangeScan()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  LocalizedObject::LocalizedObject(const Identifier& rSensorIdentifier)
    : SensorData(rSensorIdentifier)
    , m_IsGpsReadingValid(false)
    , m_IsGpsEstimateValid(false)
    , m_pGpsEstimationManager(NULL)
  {
  }

  LocalizedObject::~LocalizedObject()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  LocalizedLaserScan::LocalizedLaserScan(const Identifier& rSensorIdentifier)
    : LocalizedObject(rSensorIdentifier)
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
    kt_double nPoints = static_cast<kt_double>(rPointReadings.size());
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

  const Vector2dList& LocalizedLaserScan::GetPointReadings(kt_bool wantFiltered) const
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

  LocalizedRangeScan::LocalizedRangeScan(const Identifier& rSensorIdentifier, const std::vector<kt_double>& rReadings)
    : LocalizedLaserScan(rSensorIdentifier)
  {
    m_RangeReadings = rReadings;
  }

  LocalizedRangeScan::~LocalizedRangeScan()
  {
  }

  void LocalizedRangeScan::ComputePointReadings()
  {
    LaserRangeFinder* pLaserRangeFinder = GetLaserRangeFinder();
    if (pLaserRangeFinder == NULL)
    {
      return;
    }
    
    m_FilteredPointReadings.clear();
    m_UnfilteredPointReadings.clear();

    kt_double minimumAngle = pLaserRangeFinder->GetMinimumAngle();
    kt_double angularResolution = pLaserRangeFinder->GetAngularResolution();

    kt_double minimumRange = pLaserRangeFinder->GetMinimumRange();
    kt_double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();

    Pose2 scanPose = GetSensorPose();

    const std::vector<kt_double>& rRangeReadings = GetRangeReadings();

    // compute point readings
    kt_int32u numberOfReadings = pLaserRangeFinder->GetNumberOfRangeReadings();
    for (kt_int32u i = 0; i < numberOfReadings; i++)
    {
      kt_double angle = scanPose.GetHeading() + minimumAngle + i * angularResolution;
      kt_double rangeReading = rRangeReadings[i];

      Vector2d point;
      point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
      point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));
      m_UnfilteredPointReadings.push_back(point);

      if (math::InRange(rangeReading, minimumRange, rangeThreshold))
      {
        m_FilteredPointReadings.push_back(point);
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  LocalizedPointScan::LocalizedPointScan(const Identifier& rSensorIdentifier, const Vector2dList& rLocalPoints)
    : LocalizedLaserScan(rSensorIdentifier)
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
    if (pLaserRangeFinder == NULL)
    {
      return;
    }

    m_FilteredPointReadings.clear();
    m_UnfilteredPointReadings.clear();

    kt_double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();
    Pose2 scanPose = GetSensorPose();
    Vector2d scanPosition = scanPose.GetPosition();

    // compute point readings
    for (kt_int32u i = 0; i < m_LocalPointReadings.size(); i++)
    {
      RigidBodyTransform transform(Pose2(m_LocalPointReadings[i], 0));
      Pose2 pointPose = transform.TransformPose(scanPose);
      Vector2d point = pointPose.GetPosition();
      m_UnfilteredPointReadings.push_back(point);

      kt_double range = scanPosition.Distance(point);
      if (math::InRange(range, pLaserRangeFinder->GetMinimumRange(), rangeThreshold))
      {
        m_FilteredPointReadings.push_back(point);
      }
    }
  }

}
