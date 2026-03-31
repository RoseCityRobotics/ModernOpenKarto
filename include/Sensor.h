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

#pragma once

#ifndef __OpenKarto_Sensor_h__
#define __OpenKarto_Sensor_h__

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <Geometry.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  /**
   * Type declaration of Vector2d List
   */
  using Vector2dList = std::vector<Vector2d>;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class SensorData;

  /**
   * Abstract sensor base class
   */
  class Sensor
  {

  protected:
    /**
     * Constructs a sensor with the given name
     * @param rName sensor name
     */
    Sensor(const std::string& rName);

    /**
     * Destructor
     */
    virtual ~Sensor();

  public:
    /**
     * Gets the name of this sensor
     * @return sensor name
     */
    inline const std::string& GetName() const
    {
      return m_Name;
    }

    /**
     * Gets this sensor's offset
     * @return offset pose
     */
    inline const Pose2& GetOffsetPose() const
    {
      return m_OffsetPose;
    }

    /**
     * Sets this sensor's offset
     * @param rPose new offset pose
     */
    inline void SetOffsetPose(const Pose2& rPose)
    {
      m_OffsetPose = rPose;
    }

    /**
     * Validates this sensor
     */
    virtual void Validate() = 0;

    /**
     * Validates sensor data
     * @param pSensorData sensor data
     */
    virtual void Validate(SensorData* pSensorData) = 0;

  private:
    // restrict the following functions
    Sensor(const Sensor&);
    const Sensor& operator=(const Sensor&);

  private:
    /**
     * Sensor name
     */
    std::string m_Name;

    /**
     * Sensor offset pose
     */
    Pose2 m_OffsetPose;
  }; // Sensor

  /**
   * Type declaration of Sensor managed by std::shared_ptr
   */
  using SensorPtr = std::shared_ptr<Sensor>;

  /**
   * Type declaration of Sensor List
   */
  using SensorList = std::vector<Sensor*>;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Sensor that provides pose information relative to world coordinates.
   *
   * The user can set the offset pose of the drive sensor.  If no value is provided by the user, the default is no offset,
   * i.e, the sensor is initially at the world origin, oriented along the positive x axis.
   */
  class Drive : public Sensor
  {

  public:
    /**
     * Drive object with the given name
     * @param rName name
     */
    Drive(const std::string& rName)
      : Sensor(rName)
    {
    }

  public:
    /**
     * Destructor
     */
    virtual ~Drive()
    {
    }

    virtual void Validate()
    {
    }

    /**
     * Sensor data is valid if it is not nullptr
     */
    virtual void Validate(SensorData* pSensorData)
    {
      if (pSensorData == nullptr)
      {
        throw std::runtime_error("SensorData == nullptr");
      }
    }

  private:
    // restrict the following functions
    Drive(const Drive&);
    const Drive& operator=(const Drive&);
  }; // class Drive

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Enumerated type for valid LaserRangeFinder types
   * @cond EXCLUDE
   * If more laser range finder types are added, make sure
   * to add them to the description of LaserRangeFinder below.
   * @endcond
   */
  enum LaserRangeFinderType
  {
    LaserRangeFinder_Custom = 0,

    LaserRangeFinder_Sick_LMS100 = 1,
    LaserRangeFinder_Sick_LMS200 = 2,
    LaserRangeFinder_Sick_LMS291 = 3,

    LaserRangeFinder_Hokuyo_UTM_30LX = 4,
    LaserRangeFinder_Hokuyo_URG_04LX = 5
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class LocalizedLaserScan;
  class CoordinateConverter;
  class LaserRangeFinder;

  /**
   * The LaserRangeFinder defines a laser sensor that provides the pose offset position of a localized laser scan relative to the robot.
   * The user can set an offset pose for the sensor relative to the robot coordinate system. If no value is provided
   * by the user, the sensor is set to be at the origin of the robot coordinate system.
   * The LaserRangeFinder contains parameters for physical laser sensor used by the mapper for scan matching.
   * It also contains information about the maximum range of the sensor and provides a threshold
   * for limiting the range of readings.
   * The optimal value for the range threshold depends on the angular resolution of the scan and
   * the desired map resolution.  RangeThreshold should be set as large as possible while still
   * providing "solid" coverage between consecutive range readings.
   *
   * The following example code creates a LaserRangeFinder with default values for a Sick LMS100 laser
   * \code
   *   karto::LaserRangeFinder* pLrf = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Sick_LMS100, "/laser0");
   * \endcode
   *
   * The following laser types are supported in CreateLaserRangeFinder
   *
   *  LaserRangeFinder_Sick_LMS100
   *  LaserRangeFinder_Sick_LMS200
   *  LaserRangeFinder_Sick_LMS291
   *  LaserRangeFinder_Hokuyo_UTM_30LX
   *  LaserRangeFinder_Hokuyo_URG_04LX
   */
  class LaserRangeFinder : public Sensor
  {

  public:
    /**
     * Destructor
     */
    virtual ~LaserRangeFinder();

  public:
    /**
     * Gets this range finder sensor's minimum range
     * @return minimum range
     */
    inline double GetMinimumRange() const
    {
      return m_MinimumRange;
    }

    /**
     * Sets this range finder sensor's minimum range
     * @param minimumRange new minimum range
     */
    inline void SetMinimumRange(double minimumRange)
    {
      m_MinimumRange = minimumRange;

      SetRangeThreshold(GetRangeThreshold());
    }

    /**
     * Gets this range finder sensor's maximum range
     * @return maximum range
     */
    inline double GetMaximumRange() const
    {
      return m_MaximumRange;
    }

    /**
     * Sets this range finder sensor's maximum range
     * @param maximumRange new maximum range
     */
    inline void SetMaximumRange(double maximumRange)
    {
      m_MaximumRange = maximumRange;

      SetRangeThreshold(GetRangeThreshold());
    }

    /**
     * Gets the range threshold
     * @return range threshold
     */
    inline double GetRangeThreshold() const
    {
      return m_RangeThreshold;
    }

    /**
     * Sets the range threshold
     * @param rangeThreshold new range threshold
     */
    void SetRangeThreshold(double rangeThreshold);

    /**
     * Gets this range finder sensor's minimum angle
     * @return minimum angle
     */
    inline double GetMinimumAngle() const
    {
      return m_MinimumAngle;
    }

    /**
     * Sets this range finder sensor's minimum angle
     * @param minimumAngle new minimum angle
     */
    inline void SetMinimumAngle(double minimumAngle)
    {
      m_MinimumAngle = minimumAngle;

      Update();
    }

    /**
     * Gets this range finder sensor's maximum angle
     * @return maximum angle
     */
    inline double GetMaximumAngle() const
    {
      return m_MaximumAngle;
    }

    /**
     * Sets this range finder sensor's maximum angle
     * @param maximumAngle new maximum angle
     */
    inline void SetMaximumAngle(double maximumAngle)
    {
      m_MaximumAngle = maximumAngle;

      Update();
    }

    /**
     * Gets this range finder sensor's angular resolution
     * @return angular resolution
     */
    inline double GetAngularResolution() const
    {
      return m_AngularResolution;
    }

    /**
     * Sets this range finder sensor's angular resolution
     * @param angularResolution new angular resolution
     */
    void SetAngularResolution(double angularResolution);

    /**
     * Gets this range finder sensor's laser type
     * @return laser type of this range finder
     */
    inline LaserRangeFinderType GetType() const
    {
      return m_Type;
    }

    /**
     * Gets the number of range readings each localized range scan must contain to be a valid scan.
     * @return number of range readings
     */
    inline uint32_t GetNumberOfRangeReadings() const
    {
      return m_NumberOfRangeReadings;
    }

    /**
     * Validates this sensor
     */
    virtual void Validate();

    /**
     * Validates sensor data
     * @param pSensorData sensor data
     */
    virtual void Validate(SensorData* pSensorData);

    /**
     * Gets point readings (potentially scaling readings if given coordinate converter is not null)
     * @param pLocalizedLaserScan scan
     * @param pCoordinateConverter coordinate converter
     * @param ignoreThresholdPoints whether to ignore points that exceed the range threshold
     * @param flipY whether to flip the y-coordinate (useful for drawing applications with inverted y-coordinates)
     * @return list of points from the given scan
     */
    const Vector2dList GetPointReadings(LocalizedLaserScan* pLocalizedLaserScan, CoordinateConverter* pCoordinateConverter, bool ignoreThresholdPoints = true, bool flipY = false) const;

  public:
    /**
     * Creates a laser range finder of the given type and name
     * @param type laser type
     * @param rName name of sensor
     * @return laser range finder
     */
    [[nodiscard]] static LaserRangeFinder* CreateLaserRangeFinder(LaserRangeFinderType type, const std::string& rName);

  private:
    /**
     * Laser range finder with given name
     * @param rName name
     */
    LaserRangeFinder(const std::string& rName);

    /**
     * Sets the number of range readings based on the minimum and maximum angles of the sensor and the angular resolution
     */
    void Update()
    {
      m_NumberOfRangeReadings = static_cast<uint32_t>(std::round((GetMaximumAngle() - GetMinimumAngle()) / GetAngularResolution()) + 1);
    }

  private:
    LaserRangeFinder(const LaserRangeFinder&);
    const LaserRangeFinder& operator=(const LaserRangeFinder&);

  private:
    // sensor parameters
    double m_MinimumAngle;
    double m_MaximumAngle;

    double m_AngularResolution;

    double m_MinimumRange;
    double m_MaximumRange;

    double m_RangeThreshold;

    LaserRangeFinderType m_Type;

    uint32_t m_NumberOfRangeReadings;
  }; // LaserRangeFinder

  /**
   * Type declaration of LaserRangeFinder managed by std::shared_ptr
   */
  using LaserRangeFinderPtr = std::shared_ptr<LaserRangeFinder>;

  //@}

}

#endif // __OpenKarto_Sensor_h__
