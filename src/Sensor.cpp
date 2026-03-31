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

#include <iostream>
#include <sstream>

#include <CoordinateConverter.h>
#include <Sensor.h>
#include <SensorData.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  Sensor::Sensor(const std::string& rName)
    : m_Name(rName)
  {
  }

  Sensor::~Sensor()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  LaserRangeFinder::LaserRangeFinder(const std::string& rName)
    : Sensor(rName)
    , m_MinimumAngle(-M_PI_2)
    , m_MaximumAngle(M_PI_2)
    , m_AngularResolution(DegreesToRadians(1))
    , m_MinimumRange(0.0)
    , m_MaximumRange(80.0)
    , m_RangeThreshold(12.0)
    , m_Type(LaserRangeFinder_Custom)
    , m_NumberOfRangeReadings(0)
  {
  }

  LaserRangeFinder::~LaserRangeFinder()
  {
  }

  const Vector2dList LaserRangeFinder::GetPointReadings(LocalizedLaserScan* pLocalizedLaserScan, CoordinateConverter* pCoordinateConverter, bool ignoreThresholdPoints, bool flipY) const
  {
    Vector2dList pointReadings;

    Vector2d scanPosition = pLocalizedLaserScan->GetSensorPose().GetPosition();

    // compute point readings
    const Vector2dList& rPoints = pLocalizedLaserScan->GetPointReadings(ignoreThresholdPoints);
    for (uint32_t i = 0; i < rPoints.size(); i++)
    {
      Vector2d point = rPoints[i];

      double range = scanPosition.Distance(point);
      double clippedRange = std::clamp(range, GetMinimumRange(), GetRangeThreshold());
      if (karto::DoubleEqual(range, clippedRange) == false)
      {
        double ratio = clippedRange / range;
        point.SetX(scanPosition.GetX() + ratio * (point.GetX() - scanPosition.GetX()));
        point.SetY(scanPosition.GetY() + ratio * (point.GetY() - scanPosition.GetY()));
      }

      if (pCoordinateConverter != nullptr)
      {
        Vector2i gridPoint = pCoordinateConverter->WorldToGrid(point, flipY);
        point.SetX(gridPoint.GetX());
        point.SetY(gridPoint.GetY());
      }

      pointReadings.push_back(point);
    }

    return pointReadings;
  }

  void LaserRangeFinder::SetRangeThreshold(double rangeThreshold)
  {
    // make sure rangeThreshold is within laser range finder range
    m_RangeThreshold = std::clamp(rangeThreshold, GetMinimumRange(), GetMaximumRange());
  }

  void LaserRangeFinder::SetAngularResolution(double angularResolution)
  {
    if (m_Type == LaserRangeFinder_Custom)
    {
      m_AngularResolution = angularResolution;
    }
    else if (m_Type == LaserRangeFinder_Sick_LMS100)
    {
      if (DoubleEqual(angularResolution, DegreesToRadians(0.25)))
      {
        m_AngularResolution = DegreesToRadians(0.25);
      }
      else if (DoubleEqual(angularResolution, DegreesToRadians(0.50)))
      {
        m_AngularResolution = DegreesToRadians(0.50);
      }
      else
      {
        std::string errorMessage;
        errorMessage.append("Invalid resolution for Sick LMS100: ");
        errorMessage.append(karto::ToString(angularResolution));
        throw std::runtime_error(errorMessage);
      }
    }
    else if (m_Type == LaserRangeFinder_Sick_LMS200 || m_Type == LaserRangeFinder_Sick_LMS291)
    {
      if (DoubleEqual(angularResolution, DegreesToRadians(0.25)))
      {
        m_AngularResolution = DegreesToRadians(0.25);
      }
      else if (DoubleEqual(angularResolution, DegreesToRadians(0.50)))
      {
        m_AngularResolution = DegreesToRadians(0.50);
      }
      else if (DoubleEqual(angularResolution, DegreesToRadians(1.00)))
      {
        m_AngularResolution = DegreesToRadians(1.00);
      }
      else
      {
        std::string errorMessage;
        errorMessage.append("Invalid resolution for Sick LMS291: ");
        errorMessage.append(karto::ToString(angularResolution));
        throw std::runtime_error(errorMessage);
      }
    }
    else
    {
      throw std::runtime_error("Can't set angular resolution, please create a LaserRangeFinder of type Custom");
    }

    Update();
  }

  void LaserRangeFinder::Validate()
  {
    Update();

    // check if min < max range!
    if (GetMinimumRange() >= GetMaximumRange())
    {
      assert(false);
      throw std::runtime_error("LaserRangeFinder::Validate() - MinimumRange must be less than MaximumRange.  Please set MinimumRange and MaximumRange to valid values.");
    }

    // set range threshold to valid value
    if (InRange(GetRangeThreshold(), GetMinimumRange(), GetMaximumRange()) == false)
    {
      double newValue = std::clamp(GetRangeThreshold(), GetMinimumRange(), GetMaximumRange());
      SetRangeThreshold(newValue);
    }
  }

  void LaserRangeFinder::Validate(SensorData* pSensorData)
  {
    LocalizedRangeScan* pScan = dynamic_cast<LocalizedRangeScan*>(pSensorData);

    // verify number of range readings in laser scan matches the number of expected range
    // readings; validation only valid with LocalizedRangeScan (LocalizedPointScan may have
    // variable number of readings)
    if (pScan != nullptr && (pScan->GetNumberOfRangeReadings() != GetNumberOfRangeReadings()))
    {
      std::ostringstream errorMessage;
      errorMessage << "LaserRangeFinder::Validate() - LocalizedRangeScan contains " << pScan->GetNumberOfRangeReadings() << " range readings, expected " << GetNumberOfRangeReadings();

      //      assert(false);
      throw std::runtime_error(errorMessage.str());
    }
  }

  LaserRangeFinder* LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinderType type, const std::string& rName)
  {
    LaserRangeFinder* pLrf = nullptr;

    switch(type)
    {
      // see http://www.hizook.com/files/publications/SICK_LMS100.pdf
      // set range threshold to 18m
    case LaserRangeFinder_Sick_LMS100:
      {
        pLrf = new LaserRangeFinder((!rName.empty()) ? rName : "Sick LMS 100");

        // Sensing range is 18 meters (at 10% reflectivity, max range of 20 meters), with an error of about 20mm
        pLrf->m_MinimumRange = 0.0;
        pLrf->m_MaximumRange = 20.0;

        // 270 degree range, 50 Hz
        pLrf->m_MinimumAngle = DegreesToRadians(-135);
        pLrf->m_MaximumAngle = DegreesToRadians(135);

        // 0.25 degree angular resolution
        pLrf->m_AngularResolution = DegreesToRadians(0.25);

        pLrf->m_NumberOfRangeReadings = 1081;
      }
      break;

      // see http://www.hizook.com/files/publications/SICK_LMS200-291_Tech_Info.pdf
      // set range threshold to 10m
    case LaserRangeFinder_Sick_LMS200:
      {
        pLrf = new LaserRangeFinder((!rName.empty()) ? rName : "Sick LMS 200");

        // Sensing range is 80 meters
        pLrf->m_MinimumRange = 0.0;
        pLrf->m_MaximumRange = 80.0;

        // 180 degree range, 75 Hz
        pLrf->m_MinimumAngle = DegreesToRadians(-90);
        pLrf->m_MaximumAngle = DegreesToRadians(90);

        // 0.5 degree angular resolution
        pLrf->m_AngularResolution = DegreesToRadians(0.5);

        pLrf->m_NumberOfRangeReadings = 361;
      }
      break;

      // see http://www.hizook.com/files/publications/SICK_LMS200-291_Tech_Info.pdf
      // set range threshold to 30m
    case LaserRangeFinder_Sick_LMS291:
      {
        pLrf = new LaserRangeFinder((!rName.empty()) ? rName : "Sick LMS 291");

        // Sensing range is 80 meters
        pLrf->m_MinimumRange = 0.0;
        pLrf->m_MaximumRange = 80.0;

        // 180 degree range, 75 Hz
        pLrf->m_MinimumAngle = DegreesToRadians(-90);
        pLrf->m_MaximumAngle = DegreesToRadians(90);

        // 0.5 degree angular resolution
        pLrf->m_AngularResolution = DegreesToRadians(0.5);

        pLrf->m_NumberOfRangeReadings = 361;
      }
      break;

      // see http://www.hizook.com/files/publications/Hokuyo_UTM_LaserRangeFinder_LIDAR.pdf
      // set range threshold to 30m
    case LaserRangeFinder_Hokuyo_UTM_30LX:
      {
        pLrf = new LaserRangeFinder((!rName.empty()) ? rName : "Hokuyo UTM-30LX");

        // Sensing range is 30 meters
        pLrf->m_MinimumRange = 0.1;
        pLrf->m_MaximumRange = 30.0;

        // 270 degree range, 40 Hz
        pLrf->m_MinimumAngle = DegreesToRadians(-135);
        pLrf->m_MaximumAngle = DegreesToRadians(135);

        // 0.25 degree angular resolution
        pLrf->m_AngularResolution = DegreesToRadians(0.25);

        pLrf->m_NumberOfRangeReadings = 1081;
      }
      break;

      // see http://www.hizook.com/files/publications/HokuyoURG_Datasheet.pdf
      // set range threshold to 3.5m
    case LaserRangeFinder_Hokuyo_URG_04LX:
      {
        pLrf = new LaserRangeFinder((!rName.empty()) ? rName : "Hokuyo URG-04LX");

        // Sensing range is 4 meters. It has detection problems when scanning absorptive surfaces (such as black trimming).
        pLrf->m_MinimumRange = 0.02;
        pLrf->m_MaximumRange = 4.0;

        // 240 degree range, 10 Hz
        pLrf->m_MinimumAngle = DegreesToRadians(-120);
        pLrf->m_MaximumAngle = DegreesToRadians(120);

        // 0.352 degree angular resolution
        pLrf->m_AngularResolution = DegreesToRadians(0.352);

        pLrf->m_NumberOfRangeReadings = 751;
      }
      break;

    case LaserRangeFinder_Custom:
      {
        pLrf = new LaserRangeFinder((!rName.empty()) ? rName : "User-Defined LaserRangeFinder");

        // Sensing range is 80 meters.
        pLrf->m_MinimumRange = 0.0;
        pLrf->m_MaximumRange = 80.0;

        // 180 degree range
        pLrf->m_MinimumAngle = DegreesToRadians(-90);
        pLrf->m_MaximumAngle = DegreesToRadians(90);

        // 1.0 degree angular resolution
        pLrf->m_AngularResolution = DegreesToRadians(1.0);

        pLrf->m_NumberOfRangeReadings = 181;
      }
      break;
    }

    if (pLrf != nullptr)
    {
      pLrf->m_Type = type;

      Pose2 defaultOffset;
      pLrf->SetOffsetPose(defaultOffset);
    }

    return pLrf;
  }

}
