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

#ifndef __OpenKarto_Geometry_h__
#define __OpenKarto_Geometry_h__

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <string.h>
#include <vector>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  // Math utilities (formerly KartoMath.h)
  ////////////////////////////////////////////////////////////////////////////////////////

  /** Tolerance for floating-point comparisons */
  constexpr double KT_TOLERANCE = 1e-06;

  /** Converts degrees to radians */
  inline double DegreesToRadians(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  /** Converts radians to degrees */
  inline double RadiansToDegrees(double radians)
  {
    return radians * 180.0 / M_PI;
  }

  /** Checks whether two doubles are equal within KT_TOLERANCE */
  inline bool DoubleEqual(double a, double b)
  {
    double delta = a - b;
    return delta < 0.0 ? delta >= -KT_TOLERANCE : delta <= KT_TOLERANCE;
  }

  /**
   * Normalizes angle to be in the range of [-pi, pi]
   */
  inline double NormalizeAngle(double angle)
  {
    while (angle < -M_PI)
    {
      if (angle < -(2.0 * M_PI))
      {
        angle += (uint32_t)(angle / -(2.0 * M_PI)) * (2.0 * M_PI);
      }
      else
      {
        angle += (2.0 * M_PI);
      }
    }

    while (angle > M_PI)
    {
      if (angle > (2.0 * M_PI))
      {
        angle -= (uint32_t)(angle / (2.0 * M_PI)) * (2.0 * M_PI);
      }
      else
      {
        angle -= (2.0 * M_PI);
      }
    }

    assert(angle >= -M_PI && angle <= M_PI);

    return angle;
  }

  /**
   * Returns an equivalent angle to the first parameter such that the difference
   * when the second parameter is subtracted from this new value is an angle
   * in the normalized range of [-pi, pi].
   */
  inline double NormalizeAngleDifference(double minuend, double subtrahend)
  {
    while (minuend - subtrahend < -M_PI)
    {
      minuend += (2.0 * M_PI);
    }

    while (minuend - subtrahend > M_PI)
    {
      minuend -= (2.0 * M_PI);
    }

    return minuend;
  }

  /**
   * Square function
   */
  template<typename T>
  inline T Square(T value)
  {
    return (value * value);
  }

  /**
   * Checks whether value is in the range [a;b]
   */
  template<typename T>
  inline bool InRange(const T& value, const T& a, const T& b)
  {
    return (value >= a && value <= b);
  }

  /**
   * Checks whether value is in the range [0;maximum)
   */
  template<typename T>
  inline bool IsUpTo(const T& value, const T& maximum)
  {
    return (value >= 0 && value < maximum);
  }

  /**
   * Specialized version for uint32_t (always >= 0)
   */
  template<>
  inline bool IsUpTo<uint32_t>(const uint32_t& value, const uint32_t& maximum)
  {
    return (value < maximum);
  }

  /**
   * Aligns a value to the alignValue.
   * The alignValue should be the power of two (2, 4, 8, 16, 32 and so on)
   */
  template<class T>
  inline T AlignValue(size_t value, size_t alignValue = 8)
  {
    return static_cast<T> ((value + (alignValue - 1)) & ~(alignValue - 1));
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  // String conversion utilities (formerly StringHelper)
  ////////////////////////////////////////////////////////////////////////////////////////

  inline std::string ToString(const char* value) { return std::string(value); }

  inline std::string ToString(bool value) { return value ? "true" : "false"; }

  inline std::string ToString(int16_t value)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter << value;
    return converter.str();
  }

  inline std::string ToString(uint16_t value)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter << value;
    return converter.str();
  }

  inline std::string ToString(int32_t value)
  {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%d", value);
    return std::string(buffer);
  }

  inline std::string ToString(uint32_t value)
  {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%u", value);
    return std::string(buffer);
  }

  inline std::string ToString(int64_t value)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter << value;
    return converter.str();
  }

  inline std::string ToString(uint64_t value)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter << value;
    return converter.str();
  }

  // size_t and uint64_t are the same type on ARM64 Linux but different on macOS
#if !(defined(__linux__) && defined(__aarch64__))
  inline std::string ToString(size_t value)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter << value;
    return converter.str();
  }
#endif

  inline std::string ToString(float value)
  {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.*g", 8, (double) value);
    return std::string(buffer);
  }

  inline std::string ToString(double value)
  {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.*g", 16, value);
    return std::string(buffer);
  }

  inline std::string ToString(float value, uint32_t precision)
  {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.*f", (int32_t)precision, (double)value);
    return std::string(buffer);
  }

  inline std::string ToString(double value, uint32_t precision)
  {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.*f", (int32_t)precision, value);
    return std::string(buffer);
  }

  inline std::string ToString(const std::string& rValue) { return rValue; }

  // FromString utilities for primitives

  inline bool FromString(const std::string& rStringValue, bool& rValue)
  {
    rValue = false;
    std::string lower = rStringValue;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    if (lower == "true")
    {
      rValue = true;
    }
    return true;
  }

  inline bool FromString(const std::string& rStringValue, int16_t& rValue)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter.str(rStringValue);
    converter >> rValue;
    return true;
  }

  inline bool FromString(const std::string& rStringValue, uint16_t& rValue)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter.str(rStringValue);
    converter >> rValue;
    return true;
  }

  inline bool FromString(const std::string& rStringValue, int32_t& rValue)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter.str(rStringValue);
    converter >> rValue;
    return true;
  }

  inline bool FromString(const std::string& rStringValue, uint32_t& rValue)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter.str(rStringValue);
    converter >> rValue;
    return true;
  }

  inline bool FromString(const std::string& rStringValue, int64_t& rValue)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter.str(rStringValue);
    converter >> rValue;
    return true;
  }

  inline bool FromString(const std::string& rStringValue, uint64_t& rValue)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter.str(rStringValue);
    converter >> rValue;
    return true;
  }

  inline bool FromString(const std::string& rStringValue, float& rValue)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter.str(rStringValue);
    converter >> rValue;
    return true;
  }

  inline bool FromString(const std::string& rStringValue, double& rValue)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter.str(rStringValue);
    converter >> rValue;
    return true;
  }

  inline bool FromString(const std::string& rStringValue, std::string& rValue)
  {
    rValue = rStringValue;
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  ///** \addtogroup OpenKarto */
  //@{

  /**
   * A 2-dimensional size (width, height)
   */
  template<typename T>
  class Size2
  {
  public:
    /**
     * A size with a width and height of 0
     */
    Size2()
      : m_Width(0)
      , m_Height(0)
    {
    }

    /**
     * A size with the given width and height
     * @param width width
     * @param height height
     */
    Size2(T width, T height)
      : m_Width(width)
      , m_Height(height)
    {
    }

    /**
     * Copy constructor
     */
    Size2(const Size2& rOther)
      : m_Width(rOther.m_Width)
      , m_Height(rOther.m_Height)
    {
    }

  public:
    /**
     * Gets the width
     * @return the width
     */
    inline const T GetWidth() const
    {
      return m_Width;
    }

    /**
     * Sets the width
     * @param width
     */
    inline void SetWidth(T width)
    {
      m_Width = width;
    }

    /**
     * Gets the height
     * @return the height
     */
    inline const T GetHeight() const
    {
      return m_Height;
    }

    /**
     * Sets the height
     * @param height
     */
    inline void SetHeight(T height)
    {
      m_Height = height;
    }

    /**
     * Returns a string representation of this size
     * @return string representation of this size
     */
    inline const std::string ToString() const
    {
      std::string valueString;
      valueString.append(karto::ToString(GetWidth()));
      valueString.append(" ");
      valueString.append(karto::ToString(GetHeight()));
      return valueString;
    }

  public:
    /**
     * Assignment operator
     */
    inline Size2& operator=(const Size2& rOther)
    {
      m_Width = rOther.m_Width;
      m_Height = rOther.m_Height;

      return(*this);
    }

    /**
     * Equality operator
     */
    inline bool operator==(const Size2& rOther) const
    {
      return (m_Width == rOther.m_Width && m_Height == rOther.m_Height);
    }

    /**
     * Inequality operator
     */
    inline bool operator!=(const Size2& rOther) const
    {
      return (m_Width != rOther.m_Width || m_Height != rOther.m_Height);
    }

    /**
     * Write size onto output stream
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Size2& rSize)
    {
      rStream << rSize.ToString();
      return rStream;
    }

  private:
    T m_Width;
    T m_Height;
  }; // class Size2<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a 2-dimensional vector (x, y)
   */
  template<typename T>
  class Vector2
  {
  public:
    /**
     * Vector at the origin
     */
    Vector2()
    {
      m_Values[0] = 0;
      m_Values[1] = 0;
    }
    
    /**
     * Vector at the given location
     * @param x x
     * @param y y
     */
    Vector2(T x, T y)
    {
      m_Values[0] = x;
      m_Values[1] = y;
    }
    
  public:
    /**
     * Gets the x-coordinate of this vector
     * @return the x-coordinate of the vector
     */
    inline const T& GetX() const
    {
      return m_Values[0];
    }
    
    /**
     * Sets the x-coordinate of this vector
     * @param x the x-coordinate of the vector
     */
    inline void SetX(const T& x)
    {
      m_Values[0] = x;
    }
    
    /**
     * Gets the y-coordinate of this vector
     * @return the y-coordinate of the vector
     */
    inline const T& GetY() const
    {
      return m_Values[1];
    }
    
    /**
     * Sets the y-coordinate of this vector
     * @param y the y-coordinate of the vector
     */
    inline void SetY(const T& y)
    {
      m_Values[1] = y;
    }
        
    /**
     * Floor point operator
     * @param rOther vector
     */
    inline void MakeFloor(const Vector2& rOther)
    {
      if (rOther.m_Values[0] < m_Values[0]) m_Values[0] = rOther.m_Values[0];
      if (rOther.m_Values[1] < m_Values[1]) m_Values[1] = rOther.m_Values[1];
    }
    
    /**
     * Ceiling point operator
     * @param rOther vector
     */
    inline void MakeCeil(const Vector2& rOther)
    {
      if (rOther.m_Values[0] > m_Values[0])
      {
        m_Values[0] = rOther.m_Values[0];
      }
      
      if (rOther.m_Values[1] > m_Values[1])
      {
        m_Values[1] = rOther.m_Values[1];
      }
    }
    
    /**
     * Returns the square of the length of the vector
     * @return square of the length of the vector
     */
    inline double SquaredLength() const
    {
      return (m_Values[0] * m_Values[0]) + (m_Values[1] * m_Values[1]);
    }
    
    /**
     * Returns the length of the vector
     * @return length of the vector
     */
    inline double Length() const
    {
      return sqrt(SquaredLength());
    }
    
    /**
     * Returns the square of the distance to the given vector
     * @param rOther vector
     * @returns square of the distance to the given vector
     */
    inline double SquaredDistance(const Vector2& rOther) const
    {
      return (*this - rOther).SquaredLength();
    }
    
    /** 
     * Gets the distance to the given vector
     * @param rOther vector
     * @return distance to given vector
     */
    inline double Distance(const Vector2& rOther) const
    {
      return sqrt(SquaredDistance(rOther));
    }
    
    /**
     * Returns a string representation of this vector
     * @return string representation of this vector
     */
    inline const std::string ToString() const
    {
      std::string valueString;
      valueString.append(karto::ToString(GetX()));
      valueString.append(" ");
      valueString.append(karto::ToString(GetY()));
      return valueString;
    }

  public:
    /**
     * In-place vector addition
     */
    inline void operator+=(const Vector2& rOther)
    {
      m_Values[0] += rOther.m_Values[0];
      m_Values[1] += rOther.m_Values[1];
    }
    
    /**
     * In-place vector subtraction
     */
    inline void operator-=(const Vector2& rOther)
    {
      m_Values[0] -= rOther.m_Values[0];
      m_Values[1] -= rOther.m_Values[1];
    }

    /**
     * Vector addition
     */
    inline const Vector2 operator+(const Vector2& rOther) const
    {
      return Vector2(m_Values[0] + rOther.m_Values[0], m_Values[1] + rOther.m_Values[1]);
    }
    
    /**
     * Vector subtraction
     */
    inline const Vector2 operator-(const Vector2& rOther) const
    {
      return Vector2(m_Values[0] - rOther.m_Values[0], m_Values[1] - rOther.m_Values[1]);
    }
    
    /**
     * In-place scalar division
     */
    inline void operator/=(T scalar)
    {
      m_Values[0] /= scalar;
      m_Values[1] /= scalar;
    }
    
    /**
     * Divides a vector by the scalar
     */
    inline const Vector2 operator/(T scalar) const
    {
      return Vector2(m_Values[0] / scalar, m_Values[1] / scalar);
    }
        
    /**
     * Vector dot-product
     */
    inline double operator*(const Vector2& rOther) const
    {
      return m_Values[0] * rOther.m_Values[0] + m_Values[1] * rOther.m_Values[1];
    }    
    
    /**
     * Scales the vector by the given scalar
     */
    inline const Vector2 operator*(T scalar) const
    {
      return Vector2(m_Values[0] * scalar, m_Values[1] * scalar);
    }
    
    /**
     * Subtract the vector by the given scalar
     */
    inline const Vector2 operator-(T scalar) const
    {
      return Vector2(m_Values[0] - scalar, m_Values[1] - scalar);
    }

    /**
     * In-place scalar multiplication
     */
    inline void operator*=(T scalar)
    {
      m_Values[0] *= scalar;
      m_Values[1] *= scalar;
    }
    
    /**
     * Equality operator
     */
    inline bool operator==(const Vector2& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1]);
    }
    
    /**
     * Inequality operator
     */
    inline bool operator!=(const Vector2& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1]);
    }
    
    /**
     * Less than operator
     * @param rOther vector
     * @return true if left vector is 'less' than right vector by comparing corresponding x coordinates and then
     * corresponding y coordinates
     */
    inline bool operator<(const Vector2& rOther) const
    {
      if (m_Values[0] < rOther.m_Values[0])
      {
        return true;
      }
      else if (m_Values[0] > rOther.m_Values[0])
      {
        return false;
      }
      else 
      {
        return (m_Values[1] < rOther.m_Values[1]);
      }
    }

    /**
     * Write vector onto output stream
     */
    friend inline std::ostream& operator<<(std::ostream& rStream, const Vector2& rVector)
    {
      rStream << rVector.ToString();
      return rStream;
    }
    
  private:
    T m_Values[2];
  }; // class Vector2<T>

  /**
   * Type declaration of int32_t Vector2 as Vector2i
   */
  using Vector2i = Vector2<int32_t>;

  /**
   * Type declaration of double Vector2 as Vector2d
   */
  using Vector2d = Vector2<double>;
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a 3-dimensional vector (x, y, z)
   */
  template<typename T>
  class Vector3
  {
  public:
    /**
     * Vector at the origin
     */
    Vector3()
    {
      m_Values[0] = 0;
      m_Values[1] = 0;
      m_Values[2] = 0;
    }

    /**
     * Vector at the given location
     * @param x x
     * @param y y
     * @param z z
     */
    Vector3(T x, T y, T z)
    {
      m_Values[0] = x;
      m_Values[1] = y;
      m_Values[2] = z;
    }

    /**
     * Constructs a 3D vector from the given 2D vector with a z-component of 0
     * @param rVector 2D vector
     */
    Vector3(const Vector2<T>& rVector)
    {
      m_Values[0] = rVector.GetX();
      m_Values[1] = rVector.GetY();
      m_Values[2] = 0.0;
    }

    /**
     * Copy constructor
     */
    Vector3(const Vector3& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];
    }

  public:
    /**
     * Gets the x-component of this vector
     * @return the x-component
     */
    inline const T& GetX() const 
    {
      return m_Values[0]; 
    }

    /**
     * Sets the x-component of this vector
     * @param x x-component
     */
    inline void SetX(const T& x)
    {
      m_Values[0] = x; 
    }

    /**
     * Gets the y-component of this vector
     * @return the y-component
     */
    inline const T& GetY() const 
    {
      return m_Values[1]; 
    }

    /**
     * Sets the y-component of this vector
     * @param y y-component
     */
    inline void SetY(const T& y)
    {
      m_Values[1] = y; 
    }

    /**
     * Gets the z-component of this vector
     * @return the z-component
     */
    inline const T& GetZ() const
    {
      return m_Values[2]; 
    }

    /**
     * Sets the z-component of this vector
     * @param z z-component
     */
    inline void SetZ(const T& z)
    {
      m_Values[2] = z; 
    } 

    /**
     * Gets a 2D version of this vector (ignores z-component)
     * @return 2D version of this vector
     */
    inline Vector2<T> GetAsVector2() const
    {
      return Vector2<T>(m_Values[0], m_Values[1]);
    }

    /**
     * Floor vector operator
     * @param rOther vector
     */
    inline void MakeFloor(const Vector3& rOther)
    {
      if (rOther.m_Values[0] < m_Values[0]) m_Values[0] = rOther.m_Values[0];
      if (rOther.m_Values[1] < m_Values[1]) m_Values[1] = rOther.m_Values[1];
      if (rOther.m_Values[2] < m_Values[2]) m_Values[2] = rOther.m_Values[2];
    }

    /**
     * Ceiling vector operator
     * @param rOther vector
     */
    inline void MakeCeil(const Vector3& rOther)
    {
      if (rOther.m_Values[0] > m_Values[0]) m_Values[0] = rOther.m_Values[0];
      if (rOther.m_Values[1] > m_Values[1]) m_Values[1] = rOther.m_Values[1];
      if (rOther.m_Values[2] > m_Values[2]) m_Values[2] = rOther.m_Values[2];
    }

    /**
     * Returns the square of the length of the vector
     * @return square of the length of the vector
     */
    inline double SquaredLength() const
    {
      return (m_Values[0] * m_Values[0]) + (m_Values[1] * m_Values[1]) + (m_Values[2] * m_Values[2]);
    }

    /**
     * Returns the length of the vector.
     * @return length of the vector
     */
    inline double Length() const
    {
      return sqrt(SquaredLength());
    }
    
    /** 
     * Normalize the vector
     */ 
    inline void Normalize()
    {
      double length = Length();
      if (length > 0.0)
      {
        double inversedLength = 1.0 / length;

        m_Values[0] *= inversedLength;
        m_Values[1] *= inversedLength;
        m_Values[2] *= inversedLength;
      }
    }

    /**
     * Returns a string representation of this vector
     * @return string representation of this vector
     */
    inline const std::string ToString() const
    {
      std::string valueString;
      valueString.append(karto::ToString(GetX()));
      valueString.append(" ");
      valueString.append(karto::ToString(GetY()));
      valueString.append(" ");
      valueString.append(karto::ToString(GetZ()));
      return valueString;
    }

  public:
    /**
     * Assignment operator
     */
    inline Vector3& operator=(const Vector3& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];

      return *this;
    }

    /**
     * Vector addition
     */
    inline const Vector3 operator+(const Vector3& rOther) const
    {
      return Vector3(m_Values[0] + rOther.m_Values[0], m_Values[1] + rOther.m_Values[1], m_Values[2] + rOther.m_Values[2]);
    }

    /**
     * Vector-scalar addition
     */
    inline const Vector3 operator+(double scalar) const
    {
      return Vector3(m_Values[0] + scalar, m_Values[1] + scalar, m_Values[2] + scalar);
    }

    /**
     * Vector subtraction
     */
    inline const Vector3 operator-(const Vector3& rOther) const
    {
      return Vector3(m_Values[0] - rOther.m_Values[0], m_Values[1] - rOther.m_Values[1], m_Values[2] - rOther.m_Values[2]);
    }

    /**
     * Vector-scalar subtraction
     */
    inline const Vector3 operator-(double scalar) const
    {
      return Vector3(m_Values[0] - scalar, m_Values[1] - scalar, m_Values[2] - scalar);
    }

    /**
     * Scales the vector by the given scalar
     */
    inline const Vector3 operator*(T scalar) const
    {
      return Vector3(m_Values[0] * scalar, m_Values[1] * scalar, m_Values[2] * scalar);
    }

    /**
     * In-place vector addition
     */
    inline Vector3& operator+=(const Vector3& rOther)
    {
      m_Values[0] += rOther.m_Values[0];
      m_Values[1] += rOther.m_Values[1];
      m_Values[2] += rOther.m_Values[2];

      return *this;
    }

    /**
     * In-place vector subtraction
     */    
    inline Vector3& operator-=(const Vector3& rOther)
    {
      m_Values[0] -= rOther.m_Values[0];
      m_Values[1] -= rOther.m_Values[1];
      m_Values[2] -= rOther.m_Values[2];

      return *this;
    }

    /**
     * In-place component-wise vector multiplication
     * @param rOther vector
     * @return this vector after multiplying each component with the corresponding component in the other vector
     */
    inline Vector3& operator*=(const Vector3& rOther)
    {
      m_Values[0] *= rOther.m_Values[0];
      m_Values[1] *= rOther.m_Values[1];
      m_Values[2] *= rOther.m_Values[2];

      return *this;
    }

    /**
     * In-place component-wise vector division
     * @param rOther vector
     * @return this vector after dividing each component with the corresponding component in the other vector
     */
    inline Vector3& operator/=(const Vector3& rOther)
    {
      m_Values[0] /= rOther.m_Values[0];
      m_Values[1] /= rOther.m_Values[1];
      m_Values[2] /= rOther.m_Values[2];

      return *this;
    }

    /**
     * In-place component-wise scalar addition
     * @param rValue value to add to each component
     * @return this vector after adding each component with the given value
     */
    inline Vector3& operator+=(const T& rValue)
    {
      m_Values[0] += rValue;
      m_Values[1] += rValue;
      m_Values[2] += rValue;

      return *this;
    }

    /**
     * In-place component-wise scalar subtraction
     * @param rValue value to subtract from each component
     * @return this vector after subtracting the given value from each component
     */
    inline Vector3& operator-=(const T& rValue)
    {
      m_Values[0] -= rValue;
      m_Values[1] -= rValue;
      m_Values[2] -= rValue;

      return *this;
    }

    /**
     * In-place vector-scalar multiplication
     */    
    inline Vector3& operator*=(const T& rValue)
    {
      m_Values[0] *= rValue;
      m_Values[1] *= rValue;
      m_Values[2] *= rValue;

      return *this;
    }

    /**
     * In-place component-wise vector-scalar division
     * @param rValue value to divide from each component
     * @return this vector after dividing each component with the given value
     */
    inline Vector3& operator/=(const T& rValue)
    {
      m_Values[0] /= rValue;
      m_Values[1] /= rValue;
      m_Values[2] /= rValue;

      return *this;
    }

    /** 
     * Vector cross product
     * @param rOther vector
     * @return cross product of this vector and given vector
     */
    inline const Vector3 operator^(const Vector3& rOther) const
    {
      return Vector3(
        m_Values[1] * rOther.m_Values[2] - m_Values[2] * rOther.m_Values[1],
        m_Values[2] * rOther.m_Values[0] - m_Values[0] * rOther.m_Values[2] ,
        m_Values[0] * rOther.m_Values[1] - m_Values[1] * rOther.m_Values[0]);
    }

    /**
     * Less than operator
     * @param rOther other vector
     * @return true if left vector is 'less' than right vector by comparing corresponding x coordinates, then
     * corresponding y coordinates, and then corresponding z coordinates
     */
    inline bool operator<(const Vector3& rOther) const
    {
      if (m_Values[0] < rOther.m_Values[0])
      {
        return true;
      }
      else if (m_Values[0] > rOther.m_Values[0]) 
      {
        return false;
      }
      else if (m_Values[1] < rOther.m_Values[1])
      {
        return true;
      }
      else if (m_Values[1] > rOther.m_Values[1]) 
      {
        return false;
      }
      else
      {
        return (m_Values[2] < rOther.m_Values[2]);
      }
    }

    /**
     * Equality operator
     */
    inline bool operator==(const Vector3& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1] && m_Values[2] == rOther.m_Values[2]);
    }

    /**
     * Inequality operator
     */
    inline bool operator!=(const Vector3& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1] || m_Values[2] != rOther.m_Values[2]);
    }

    /**
     * Write vector onto output stream
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Vector3& rVector)
    {
      rStream << rVector.ToString();
      return rStream;
    }

  private:
    T m_Values[3];
  }; // class Vector3<T>

  /**
   * Type declaration of int32_t Vector3 as Vector3i
   */
  using Vector3i = Vector3<int32_t>;

  /**
   * Type declaration of double Vector3 as Vector3d
   */
  using Vector3d = Vector3<double>;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a 2-dimensional bounding box.
   */
  class BoundingBox2
  {
  public:
    /*
     * Bounding box of maximal size
     */
    BoundingBox2();

    /**
     * Bounding box with given minimum and maximum values.
     * @param rMinimum minimum value
     * @param rMaximum maximum value
     */
    BoundingBox2(const Vector2d& rMinimum, const Vector2d& rMaximum);

  public:
    /** 
     * Get bounding box minimum
     * @return bounding box minimum
     */
    inline const Vector2d& GetMinimum() const
    { 
      return m_Minimum; 
    }

    /** 
     * Set bounding box minimum
     * @param rMinimum bounding box minimum
     */
    inline void SetMinimum(const Vector2d& rMinimum)
    {
      m_Minimum = rMinimum;
    }

    /** 
     * Get bounding box maximum
     * @return bounding box maximum
     */
    inline const Vector2d& GetMaximum() const
    { 
      return m_Maximum;
    }

    /** 
     * Set bounding box maximum
     * @param rMaximum bounding box maximum
     */
    inline void SetMaximum(const Vector2d& rMaximum)
    {
      m_Maximum = rMaximum;
    }

    /**
     * Get the size of this bounding box
     * @return bounding box size
     */
    inline Size2<double> GetSize() const
    {
      Vector2d size = m_Maximum - m_Minimum;

      return Size2<double>(size.GetX(), size.GetY());
    }

    /**
     * Add vector to bounding box
     * @param rPoint point
     */
    inline void Add(const Vector2d& rPoint)
    {
      m_Minimum.MakeFloor(rPoint);
      m_Maximum.MakeCeil(rPoint);
    }

    /**
     * Add given bounding box to this bounding box
     * @param rBoundingBox bounding box
     */
    inline void Add(const BoundingBox2& rBoundingBox)
    {
      Add(rBoundingBox.GetMinimum());
      Add(rBoundingBox.GetMaximum());
    }
    
    /**
     * Whether the given point is in the bounds of this box
     * @param rPoint point
     * @return whether the given point is in the bounds of this box
     */
    inline bool Contains(const Vector2d& rPoint) const
    {
      return (InRange(rPoint.GetX(), m_Minimum.GetX(), m_Maximum.GetX()) &&
              InRange(rPoint.GetY(), m_Minimum.GetY(), m_Maximum.GetY()));
    }

    /** 
     * Checks if this bounding box intersects with the given bounding box
     * @param rOther bounding box
     * @return true if this bounding box intersects with the given bounding box
     */
    inline bool Intersects(const BoundingBox2& rOther) const
    { 
      if ((m_Maximum.GetX() < rOther.m_Minimum.GetX()) || (m_Minimum.GetX() > rOther.m_Maximum.GetX()))
      {
        return false;
      }
      
      if ((m_Maximum.GetY() < rOther.m_Minimum.GetY()) || (m_Minimum.GetY() > rOther.m_Maximum.GetY()))
      {
        return false;
      }

      return true;
    }

    /** 
     * Checks if this bounding box contains the given bounding box
     * @param rOther bounding box
     * @return true if this bounding box contains the given bounding box
     */
    inline bool Contains(const BoundingBox2& rOther) const
    {
      if ((m_Maximum.GetX() < rOther.m_Minimum.GetX()) || (m_Minimum.GetX() > rOther.m_Maximum.GetX()))
      {
        return false;
      }
      
      if ((m_Maximum.GetY() < rOther.m_Minimum.GetY()) || (m_Minimum.GetY() > rOther.m_Maximum.GetY()))
      {
        return false;
      }
      
      if ((m_Minimum.GetX() <= rOther.m_Minimum.GetX()) && (rOther.m_Maximum.GetX() <= m_Maximum.GetX()) && 
          (m_Minimum.GetY() <= rOther.m_Minimum.GetY()) && (rOther.m_Maximum.GetY() <= m_Maximum.GetY()))
      {
        return true;
      }

      return false;
    }

  private:
    Vector2d m_Minimum;
    Vector2d m_Maximum;
  }; // class BoundingBox2

  /**
   * Stores x, y, width and height that represents the location and size of a rectangle.
   * Note that (x, y) is at bottom-left in the mapper!
   */
  template<typename T>
  class Rectangle2
  {
  public:
    /**
     * Rectangle with all parameters set to 0
     */
    Rectangle2()
    {
    }

    /**
     * Rectangle with given parameters
     * @param x x-coordinate of left edge of rectangle
     * @param y y-coordinate of bottom edge of rectangle
     * @param width width of rectangle
     * @param height height of rectangle
     */
    Rectangle2(T x, T y, T width, T height)
      : m_Position(x, y)
      , m_Size(width, height)
    {
    }

    /**
     * Rectangle with given position and size
     * @param rPosition (x,y)-coordinate of rectangle
     * @param rSize size of the rectangle
     */
    Rectangle2(const Vector2<T>& rPosition, const Size2<T>& rSize)
      : m_Position(rPosition)
      , m_Size(rSize)
    {
    }

    /**
     * Rectangle with given top-left and bottom-right coordinates
     * @param rTopLeft top-left (x,y)-coordinate of rectangle
     * @param rBottomRight bottom-right (x,y)-coordinate of rectangle
     */
    Rectangle2(const Vector2<T>& rTopLeft, const Vector2<T>& rBottomRight)
      : m_Position(rTopLeft)
      , m_Size(rBottomRight.GetX() - rTopLeft.GetX(), rBottomRight.GetY() - rTopLeft.GetY())
    {
    }

    /**
     * Copy constructor
     */
    Rectangle2(const Rectangle2& rOther)
      : m_Position(rOther.m_Position)
      , m_Size(rOther.m_Size)
    {
    }

  public:
    /**
     * Gets the x-coordinate of the left edge of this rectangle
     * @return the x-coordinate of the left edge of this rectangle
     */
    inline T GetX() const
    {
      return m_Position.GetX();
    }
    
    /**
     * Sets the x-coordinate of the left edge of this rectangle
     * @param rX new x-coordinate for the left edge
     */
    inline void SetX(const T& rX) 
    {
      m_Position.SetX(rX);
    }

    /**
     * Gets the y-coordinate of the bottom edge of this rectangle
     * @return the y-coordinate of the bottom edge of this rectangle
     */
    inline T GetY() const
    {
      return m_Position.GetY();
    }

    /**
     * Sets the y-coordinate of the bottom edge of this rectangle
     * @param rY new y-coordinate for the bottom edge
     */
    inline void SetY(const T& rY)
    {
      m_Position.SetY(rY);
    }
    
    /**
     * Gets the width of this rectangle
     * @return the width of this rectangle
     */
    inline T GetWidth() const
    {
      return m_Size.GetWidth();
    }

    /**
     * Sets the width of this rectangle
     * @param rWidth new width
     */
    inline void SetWidth(const T& rWidth)
    {
      m_Size.SetWidth(rWidth);
    }
    
    /**
     * Gets the height of this rectangle
     * @return the height of this rectangle
     */
    inline T GetHeight() const
    {
      return m_Size.GetHeight();
    }

    /**
     * Sets the height of this rectangle
     * @param rHeight new height
     */
    inline void SetHeight(const T& rHeight)
    {
      m_Size.SetHeight(rHeight);
    }
    
    /**
     * Gets the position of this rectangle
     * @return the position of this rectangle
     */    
    inline const Vector2<T>& GetPosition() const
    {
      return m_Position;
    }

    /**
     * Sets the position of this rectangle
     * @param rX new x position
     * @param rY new y position
     */    
    inline void SetPosition(const T& rX, const T& rY)
    {
      m_Position = Vector2<T>(rX, rY);
    }

    /**
     * Sets the position of this rectangle
     * @param rPosition new position
     */    
    inline void SetPosition(const Vector2<T>& rPosition)
    {
      m_Position = rPosition;
    }

    /**
     * Gets the size of this rectangle
     * @return the size of this rectangle
     */    
    inline const Size2<T>& GetSize() const
    {
      return m_Size;
    }

    /**
     * Sets the size of this rectangle
     * @param rSize new size
     */    
    inline void SetSize(const Size2<T>& rSize) 
    {
      m_Size = rSize;
    }

    /**
     * Get the left coordinate of this rectangle
     * @return left coordinate of this rectangle
     */
    inline T GetLeft() const
    {
      return m_Position.GetX();
    }

    /**
     * Set the left coordinate of this rectangle
     * param rLeft new left coordinate of this rectangle
     */
    inline void SetLeft(const T& rLeft)
    {
      m_Position.SetX(rLeft);
    }

    /**
     * Get the top coordinate of this rectangle
     * @return top coordinate of this rectangle
     */
    inline T GetTop() const
    {
      return m_Position.GetY();
    }

    /**
     * Set the top coordinate of this rectangle
     * param rTop new top coordinate of this rectangle
     */
    inline void SetTop(const T& rTop)
    {
      m_Position.SetY(rTop);
    }

    /**
     * Get the right coordinate of this rectangle
     * @return right coordinate of this rectangle
     */
    inline T GetRight() const
    {
      return m_Position.GetX() + m_Size.GetWidth();
    }

    /**
     * Set the right coordinate of this rectangle
     * param rRight new right coordinate of this rectangle
     */
    inline void SetRight(const T& rRight)
    {
      m_Size.SetWidth(rRight - m_Position.GetX());
    }

    /**
     * Get the bottom coordinate of this rectangle
     * @return bottom coordinate of this rectangle
     */
    inline T GetBottom() const
    {
      return m_Position.GetY() + m_Size.GetHeight();
    }

    /**
     * Set the bottom coordinate of this rectangle
     * param rBottom new bottom coordinate of this rectangle
     */
    inline void SetBottom(const T& rBottom)
    {
      m_Size.SetHeight(rBottom - m_Position.GetY());
    }

    /**
     * Get the top-left coordinate of this rectangle
     * @return top-left coordinate of this rectangle
     */
    inline Vector2<T> GetTopLeft()
    {
      return Vector2<T>(GetLeft(), GetTop());
    }

    /**
     * Get the top-right coordinate of this rectangle
     * @return top-right coordinate of this rectangle
     */
    inline Vector2<T> GetTopRight()
    {
      return Vector2<T>(GetRight(), GetTop());
    }

    /**
     * Get the bottom-left coordinate of this rectangle
     * @return bottom-left coordinate of this rectangle
     */
    inline Vector2<T> GetBottomLeft()
    {
      return Vector2<T>(GetLeft(), GetBottom());
    }

    /**
     * Get the bottom-right coordinate of this rectangle
     * @return bottom-right coordinate of this rectangle
     */
    inline Vector2<T> GetBottomRight()
    {
      return Vector2<T>(GetRight(), GetBottom());
    }

    /**
     * Gets the center of this rectangle
     * @return the center of this rectangle
     */
    inline const Vector2<T> GetCenter() const
    {
      return Vector2<T>(m_Position.GetX() + m_Size.GetWidth() * 0.5, m_Position.GetY() + m_Size.GetHeight() * 0.5);
    }

    /**
     * Whether this rectangle contains the given rectangle
     * @param rOther rectangle
     * @return true if this rectangle contains the given rectangle, false otherwise
     */
    inline bool Contains(const Rectangle2<T>& rOther)
    {
      T l1 = m_Position.GetX();
      T r1 = m_Position.GetX();
      if (m_Size.GetWidth() < 0)
        l1 += m_Size.GetWidth();
      else
        r1 += m_Size.GetWidth();
      if (l1 == r1) // null rect
        return false;
      
      T l2 = rOther.m_Position.GetX();
      T r2 = rOther.m_Position.GetX();
      if (rOther.m_Size.GetWidth() < 0)
        l2 += rOther.m_Size.GetWidth();
      else
        r2 += rOther.m_Size.GetWidth();
      if (l2 == r2) // null rect
        return false;

      if (l2 < l1 || r2 > r1)
        return false;

      T t1 = m_Position.GetY();
      T b1 = m_Position.GetY();
      if (m_Size.GetHeight() < 0)
        t1 += m_Size.GetHeight();
      else
        b1 += m_Size.GetHeight();
      if (t1 == b1) // null rect
        return false;

      T t2 = rOther.m_Position.GetY();
      T b2 = rOther.m_Position.GetY();
      if (rOther.m_Size.GetHeight() < 0)
        t2 += rOther.m_Size.GetHeight();
      else
        b2 += rOther.m_Size.GetHeight();
      if (t2 == b2) // null rect
        return false;

      if (t2 < t1 || b2 > b1)
        return false;

      return true;
    }

  public:
    /**
     * Assignment operator
     */
    Rectangle2& operator = (const Rectangle2& rOther)
    {
      m_Position = rOther.m_Position;
      m_Size = rOther.m_Size;
      
      return *this;
    }

    /**
     * Equality operator
     */
    inline bool operator == (const Rectangle2& rOther) const
    {
      return (m_Position == rOther.m_Position && m_Size == rOther.m_Size);
    }

    /**
     * Inequality operator
     */
    inline bool operator != (const Rectangle2& rOther) const
    {
      return (m_Position != rOther.m_Position || m_Size != rOther.m_Size);
    }

  private:
    Vector2<T> m_Position;
    Size2<T> m_Size;
  }; // class Rectangle2<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a pose (position and heading) in 2-dimensional space.
   */
  class Pose2
  {
  public:
    /**
     * Pose at the origin with a heading of 0
     */
    Pose2();

    /**
     * Pose with given position and heading
     * @param rPosition position
     * @param heading heading
     **/
    Pose2(const Vector2d& rPosition, double heading = 0);

    /**
     * Pose with given position and heading
     * @param x x-coordinate
     * @param y y-coordinate
     * @param heading heading
     **/
    Pose2(double x, double y, double heading);

    /**
     * Copy constructor
     */
    Pose2(const Pose2& rOther);

  public:
    /**
     * Returns the x-coordinate
     * @return the x-coordinate of this pose
     */
    inline double GetX() const
    {
      return m_Position.GetX();
    }

    /**
     * Sets the x-coordinate
     * @param x new x-coordinate
     */
    inline void SetX(double x)
    {
      m_Position.SetX(x);
    }

    /**
     * Returns the y-coordinate
     * @return the y-coordinate of this pose
     */
    inline double GetY() const
    {
      return m_Position.GetY();
    }

    /**
     * Sets the y-coordinate
     * @param y new y-coordinate
     */
    inline void SetY(double y)
    {
      m_Position.SetY(y);
    }

    /**
     * Returns the position
     * @return the position of this pose
     */
    inline const Vector2d& GetPosition() const
    {
      return m_Position;
    }

    /**
     * Sets the position
     * @param rPosition new position
     */
    inline void SetPosition(const Vector2d& rPosition)
    {
      m_Position = rPosition;
    }

    /**
     * Returns the heading of the pose (in radians)
     * @return the heading of this pose
     */
    inline double GetHeading() const
    {
      return m_Heading;
    }

    /**
     * Sets the heading
     * @param heading new heading
     */
    inline void SetHeading(double heading)
    {
      m_Heading = heading;
    }

    /** 
     * Return the squared distance between this pose and the given pose
     * @param rOther pose
     * @return squared distance between this pose and the given pose
     */
    inline double SquaredDistance(const Pose2& rOther) const
    {
      return m_Position.SquaredDistance(rOther.m_Position);
    }
    
    /**
     * Return the angle from this pose to the given vector
     * @param rVector vector
     * @return angle to given vector
     */
    inline double AngleTo(const Vector2d& rVector) const
    {
      double angle = atan2(rVector.GetY() - GetY(), rVector.GetX() - GetX());
      return karto::NormalizeAngle(angle - GetHeading());
    }
    
    /**
     * Returns a string representation of this pose
     * @param precision precision, default 4
     * @return string representation of this pose
     */
    inline const std::string ToString(uint32_t precision = 4) const
    {
      std::string valueString;
      valueString.append(karto::ToString(m_Position.GetX(), precision));
      valueString.append(" ");
      valueString.append(karto::ToString(m_Position.GetY(), precision));
      valueString.append(" ");
      valueString.append(karto::ToString(m_Heading, precision));
      return valueString;
    }

  public:
    /**
     * Assignment operator
     */
    inline Pose2& operator = (const Pose2& rOther)
    {
      m_Position = rOther.m_Position;
      m_Heading = rOther.m_Heading;

      return *this;
    }

    /**
     * Equality operator
     */
    inline bool operator==(const Pose2& rOther) const
    {
      return (m_Position == rOther.m_Position && m_Heading == rOther.m_Heading);
    }

    /**
     * Inequality operator
     */
    inline bool operator!=(const Pose2& rOther) const
    {
      return (m_Position != rOther.m_Position || m_Heading != rOther.m_Heading);
    }

    /**
     * In-place Pose2 addition
     */
    inline void operator+=(const Pose2& rOther)
    {
      m_Position += rOther.m_Position;
      m_Heading = NormalizeAngle(m_Heading + rOther.m_Heading);
    }

    /**
     * Pose2 addition
     */
    inline Pose2 operator+(const Pose2& rOther) const
    {
      return Pose2(m_Position + rOther.m_Position, NormalizeAngle(m_Heading + rOther.m_Heading));
    }

    /**
     * Pose2 subtraction
     */
    inline Pose2 operator-(const Pose2& rOther) const
    {
      return Pose2(m_Position - rOther.m_Position, NormalizeAngle(m_Heading - rOther.m_Heading));
    }

    /**
     * Write this pose onto output stream
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Pose2& rPose)
    {
      rStream << rPose.ToString();
      return rStream;
    }
    
  private:
    Vector2d m_Position;

    double m_Heading;
  }; // class Pose2

  /**
   * Type declaration of Pose2 List
   */
  using Pose2List = std::vector<Pose2>;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a 3x3 matrix
   */
  class Matrix3
  {
  public:
    /**
     * Matrix with all elements as 0
     */
    Matrix3()
    {
      Clear();
    }

    /**
     * Copy constructor
     */
    inline Matrix3(const Matrix3& rOther)
    {
      memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(double));
    }

  public:
    /**
     * Sets this matrix to the identity matrix
     */
    void SetToIdentity()
    {
      memset(m_Matrix, 0, 9*sizeof(double));

      for (int32_t i = 0; i < 3; i++)
      {
        m_Matrix[i][i] = 1.0;
      }
    }

    /**
     * Sets this matrix to zero matrix
     */
    void Clear()
    {
      memset(m_Matrix, 0, 9*sizeof(double));
    }

    /**
     * Sets this matrix to be the rotation matrix of a rotation around the given axis
     * @param x x-coordinate of axis
     * @param y y-coordinate of axis
     * @param z z-coordinate of axis
     * @param radians amount of rotation
     */
    void FromAxisAngle(double x, double y, double z, const double radians)
    {
      double cosRadians = cos(radians);
      double sinRadians = sin(radians);
      double oneMinusCos = 1.0 - cosRadians;

      double xx = x * x;
      double yy = y * y;
      double zz = z * z;

      double xyMCos = x * y * oneMinusCos;
      double xzMCos = x * z * oneMinusCos;
      double yzMCos = y * z * oneMinusCos;

      double xSin = x * sinRadians;
      double ySin = y * sinRadians;
      double zSin = z * sinRadians;

      m_Matrix[0][0] = xx * oneMinusCos + cosRadians;
      m_Matrix[0][1] = xyMCos - zSin;
      m_Matrix[0][2] = xzMCos + ySin;

      m_Matrix[1][0] = xyMCos + zSin;
      m_Matrix[1][1] = yy * oneMinusCos + cosRadians;
      m_Matrix[1][2] = yzMCos - xSin;

      m_Matrix[2][0] = xzMCos - ySin;
      m_Matrix[2][1] = yzMCos + xSin;
      m_Matrix[2][2] = zz * oneMinusCos + cosRadians;
    }

    /**
     * Returns transposed version of this matrix
     * @return transposed matrix
     */
    Matrix3 Transpose() const
    {
      Matrix3 transpose;

      for (uint32_t row = 0; row < 3; row++)
      {
        for (uint32_t col = 0; col < 3; col++)
        {
          transpose.m_Matrix[row][col] = m_Matrix[col][row];
        }
      }

      return transpose;
    }

    /**
     * Returns the inverse of the matrix
     * @return matrix inverse
     */
    Matrix3 Inverse() const
    {
      Matrix3 kInverse = *this;
      bool haveInverse = InverseFast(kInverse, 1e-14);
      if (haveInverse == false)
      {
        assert(false);
      }
      return kInverse;
    }
    
    /**
     * Returns a string representation of this matrix
     * @return string representation of this matrix
     */
    inline const std::string ToString() const
    {
      std::string valueString;

      for (int row = 0; row < 3; row++)
      {
        for (int col = 0; col < 3; col++)
        {
          valueString.append(karto::ToString(m_Matrix[row][col]));
          valueString.append(" ");
        }
      }
      
      return valueString;
    }

  public:
    /**
     * Assignment operator
     */
    inline Matrix3& operator = (const Matrix3& rOther)
    {
      memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(double));
      return *this;
    }

    /**
     * Matrix element access, allows use of construct mat(r, c)
     * @param row row
     * @param column column
     * @return reference to mat(r,c)
     */
    inline double& operator()(uint32_t row, uint32_t column)
    {
      return m_Matrix[row][column];
    }

    /**
     * Read-only matrix element access, allows use of construct mat(r, c)
     * @param row row
     * @param column column
     * @return element at mat(r,c)
     */
    inline double operator()(uint32_t row, uint32_t column) const
    {
      return m_Matrix[row][column];
    }

    /**
     * Matrix multiplication
     */
    Matrix3 operator*(const Matrix3& rOther) const
    {
      Matrix3 product;

      for (size_t row = 0; row < 3; row++)
      {
        for (size_t col = 0; col < 3; col++)
        {
          product.m_Matrix[row][col] = m_Matrix[row][0]*rOther.m_Matrix[0][col] + m_Matrix[row][1]*rOther.m_Matrix[1][col] + m_Matrix[row][2]*rOther.m_Matrix[2][col];
        }
      }

      return product;
    }

    /**
     * Matrix3 and Pose2 multiplication: matrix * pose [3x3 * 3x1 = 3x1]
     */
    inline Pose2 operator*(const Pose2& rPose2) const
    {
      Pose2 pose2;

      pose2.SetX(m_Matrix[0][0] * rPose2.GetX() + m_Matrix[0][1] * rPose2.GetY() + m_Matrix[0][2] * rPose2.GetHeading());
      pose2.SetY(m_Matrix[1][0] * rPose2.GetX() + m_Matrix[1][1] * rPose2.GetY() + m_Matrix[1][2] * rPose2.GetHeading());
      pose2.SetHeading(m_Matrix[2][0] * rPose2.GetX() + m_Matrix[2][1] * rPose2.GetY() + m_Matrix[2][2] * rPose2.GetHeading());

      return pose2;
    }

    /**
     * In-place matrix addition
     */
    inline void operator+=(const Matrix3& rkMatrix)
    {
      for (uint32_t row = 0; row < 3; row++)
      {
        for (uint32_t col = 0; col < 3; col++)
        {
          m_Matrix[row][col] += rkMatrix.m_Matrix[row][col];
        }
      }
    }
    
    /**
     * Equality operator
     */
    inline bool operator==(const Matrix3& rkMatrix)
    {
      for (uint32_t row = 0; row < 3; row++)
      {
        for (uint32_t col = 0; col < 3; col++)
        {
          if (DoubleEqual(m_Matrix[row][col], rkMatrix.m_Matrix[row][col]) == false)
          {
            return false;
          }
        }
      }
      
      return true;
    }
    
    /**
     * Write matrix onto output stream
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Matrix3& rMatrix)
    {
      rStream << rMatrix.ToString();
      return rStream;
    }

  private:
    /**
     * Internal helper method for inverse matrix calculation
     * This code is from the OgreMatrix3 class
     * @param rkInverse output parameter
     * @param fTolerance tolerance
     * @return whether this matrix was successfully inverted; if so,
     * the inverse will be in rkInverse
     */
    bool InverseFast(Matrix3& rkInverse, double fTolerance = KT_TOLERANCE) const
    {
      // Invert a 3x3 using cofactors.  This is about 8 times faster than
      // the Numerical Recipes code which uses Gaussian elimination.
      rkInverse.m_Matrix[0][0] = m_Matrix[1][1]*m_Matrix[2][2] - m_Matrix[1][2]*m_Matrix[2][1];
      rkInverse.m_Matrix[0][1] = m_Matrix[0][2]*m_Matrix[2][1] - m_Matrix[0][1]*m_Matrix[2][2];
      rkInverse.m_Matrix[0][2] = m_Matrix[0][1]*m_Matrix[1][2] - m_Matrix[0][2]*m_Matrix[1][1];
      rkInverse.m_Matrix[1][0] = m_Matrix[1][2]*m_Matrix[2][0] - m_Matrix[1][0]*m_Matrix[2][2];
      rkInverse.m_Matrix[1][1] = m_Matrix[0][0]*m_Matrix[2][2] - m_Matrix[0][2]*m_Matrix[2][0];
      rkInverse.m_Matrix[1][2] = m_Matrix[0][2]*m_Matrix[1][0] - m_Matrix[0][0]*m_Matrix[1][2];
      rkInverse.m_Matrix[2][0] = m_Matrix[1][0]*m_Matrix[2][1] - m_Matrix[1][1]*m_Matrix[2][0];
      rkInverse.m_Matrix[2][1] = m_Matrix[0][1]*m_Matrix[2][0] - m_Matrix[0][0]*m_Matrix[2][1];
      rkInverse.m_Matrix[2][2] = m_Matrix[0][0]*m_Matrix[1][1] - m_Matrix[0][1]*m_Matrix[1][0];
      
      double fDet = m_Matrix[0][0]*rkInverse.m_Matrix[0][0] + m_Matrix[0][1]*rkInverse.m_Matrix[1][0]+ m_Matrix[0][2]*rkInverse.m_Matrix[2][0];
      
      if (fabs(fDet) <= fTolerance)
      {
        return false;
      }
      
      double fInvDet = 1.0/fDet;
      for (size_t row = 0; row < 3; row++)
      {
        for (size_t col = 0; col < 3; col++)
        {
          rkInverse.m_Matrix[row][col] *= fInvDet;
        }
      }
      
      return true;
    }
    
  private:
    double m_Matrix[3][3];
  }; // class Matrix3

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  namespace gps
  {
    /**
     * Defines a GPS point
     */
    class PointGps : public karto::Vector2d
    {
    public:
      /**
       * GPS point with longitude and latitude of (0, 0)
       */
      PointGps()
      {
      }
      
      /**
       * Construct GPS point with given longitude and latitude
       * @param latitude latitude
       * @param longitude longitude
       */
      PointGps(double latitude, double longitude)
      {
        SetX(longitude);
        SetY(latitude);
      }
      
      /**
       * Copy constructor
       */
      PointGps(const PointGps& rOther)
      {
        SetX(rOther.GetLongitude());
        SetY(rOther.GetLatitude());
      }
      
    public:
      /**
       * Gets the latitude
       * @return latitude
       */
      inline double GetLatitude() const
      {
        return GetY();
      }
      
      /**
       * Sets the latitude
       * @param latitude new latitude
       */
      inline void SetLatitude(double latitude)
      {
        SetY(latitude);
      }
      
      /**
       * Gets the longitude
       * @return longitude
       */
      inline double GetLongitude() const
      {
        return GetX();
      }
      
      /**
       * Sets the longitude 
       * @param longitude new longitude
       */
      inline void SetLongitude(double longitude)
      {
        SetX(longitude);
      }
      
      /**
       * Returns bearing to given GPS point
       * Reference: http://www.movable-type.co.uk/scripts/latlong.html
       * @param rOther GPS point
       * @return bearing to given GPS point
       */
      inline double GetBearing(const PointGps& rOther)
      {
        double lat1 = DegreesToRadians(GetLatitude());
        double long1 = DegreesToRadians(GetLongitude());
        double lat2 = DegreesToRadians(rOther.GetLatitude());
        double long2 = DegreesToRadians(rOther.GetLongitude());
        
        double deltaLong = long2 - long1;
        
        double y = sin(deltaLong) * cos(lat2);
        double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLong);
        
        return RadiansToDegrees(atan2(y, x));
      }
      
      /**
       * Adds the given GPS offset to this GPS point
       * @param rOffset amount to offset this GPS point by
       */
      void AddOffset(const PointGps& rOffset)
      {
        AddOffset(rOffset.GetLatitude(), rOffset.GetLongitude());
      }
      
      /**
       * Adds the given GPS offset to this GPS point
       * @param latitude amount to offset this GPS point by in the latitude direction
       * @param longitude amount to offset this GPS point by in the longitude direction
       */
      void AddOffset(double latitude, double longitude)
      {
        SetX(GetLongitude() + longitude);
        SetY(GetLatitude() - latitude);
      }
      
      /**
       * Distance between this GPS point and the given GPS point
       * Reference: http://code.google.com/p/geopy/wiki/GettingStarted
       * @param rOther GPS point
       * @return distance between this point and the given point
       */
      double Distance(const PointGps& rOther)
      {
        double dlon = rOther.GetLongitude() - GetLongitude();

        const double slat1 = sin(DegreesToRadians(GetLatitude()));
        const double clat1 = cos(DegreesToRadians(GetLatitude()));

        const double slat2 = sin(DegreesToRadians(rOther.GetLatitude()));
        const double clat2 = cos(DegreesToRadians(rOther.GetLatitude()));

        const double sdlon = sin(DegreesToRadians(dlon));
        const double cdlon = cos(DegreesToRadians(dlon));

        const double t1 = clat2 * sdlon;
        const double t2 = clat1 * slat2 - slat1 * clat2 * cdlon;
        const double t3 = slat1 * slat2 + clat1 * clat2 * cdlon;
        const double dist = atan2(sqrt(t1*t1 + t2*t2), t3);

        const double earthRadius = 6372.795;
        return dist * earthRadius;
      }

      /**
       * String representation of this GPS point
       * @return string representation of this GPS point
       */
      inline const std::string ToString() const
      {
        std::string valueString;
        valueString.append(karto::ToString(GetLatitude()));
        valueString.append(" ");
        valueString.append(karto::ToString(GetLongitude()));
        return valueString;
      }
      
      /**
       * Write this GPS point onto output stream
       */
      friend inline std::ostream& operator << (std::ostream& rStream, const PointGps& rPointGps)
      {
        rStream << rPointGps.ToString();
        return rStream;
      }

    public:
      /**
       * Returns a new GPS point that is the result of component-wise addition of this point and the given point
       * @param rOther GPS point
       * @return new point that is the component-wise addition of this point and the given point
       */
      inline const PointGps operator + (const PointGps& rOther) const
      {
        return PointGps(GetLatitude() + rOther.GetLatitude(), GetLongitude() + rOther.GetLongitude());
      }
      
      /**
       * Returns a new GPS coordinate that is the result of component-wise subtraction of the given point from this point
       * @param rOther GPS point
       * @return new point that is the component-wise subtracion of the given point from this point
       */
      inline const PointGps operator - (const PointGps& rOther) const
      {
        return PointGps(GetLatitude() - rOther.GetLatitude(), GetLongitude() - rOther.GetLongitude());
      }
    };

    using PointGpsList = std::vector<PointGps>;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  inline std::string ToString(const Size2<T>& rValue)
  {
    return rValue.ToString();
  }

  template<typename T>
  inline std::string ToString(const Vector2<T>& rValue)
  {
    return rValue.ToString();
  }

  template<typename T>
  inline std::string ToString(const Vector3<T>& rValue)
  {
    return rValue.ToString();
  }

  inline std::string ToString(const Pose2& rValue)
  {
    return rValue.ToString();
  }

  template<typename T>
  inline bool FromString(const std::string& rStringValue, Size2<T>& rValue)
  {
    size_t index = rStringValue.find_first_of(" ");
    if (index != std::string::npos)
    {
      std::string stringValue;
      T value;

      stringValue = rStringValue.substr(0, index);
      value = 0;
      FromString(stringValue, value);
      rValue.SetWidth(value);

      stringValue = rStringValue.substr(index + 1, rStringValue.size());
      value = 0;
      FromString(stringValue, value);
      rValue.SetHeight(value);

      return true;
    }
    return false;
  }

  template<typename T>
  inline bool FromString(const std::string& rStringValue, Vector2<T>& rValue)
  {
    size_t index = rStringValue.find_first_of(" ");
    if (index != std::string::npos)
    {
      std::string stringValue;
      T value;

      stringValue = rStringValue.substr(0, index);
      value = 0;
      FromString(stringValue, value);
      rValue.SetX(value);

      stringValue = rStringValue.substr(index + 1, rStringValue.size());
      value = 0;
      FromString(stringValue, value);
      rValue.SetY(value);

      return true;
    }
    return false;
  }

  template<typename T>
  inline bool FromString(const std::string& rStringValue, Vector3<T>& rValue)
  {
    std::string tempString = rStringValue;
    size_t index = tempString.find_first_of(" ");
    if (index != std::string::npos)
    {
      std::string stringValue;
      T value;

      // Get X
      stringValue = tempString.substr(0, index);

      value = 0;
      FromString(stringValue, value);
      rValue.SetX(value);

      // Get Y
      tempString = rStringValue.substr(index + 1, rStringValue.size());
      index = tempString.find_first_of(" ");

      stringValue = tempString.substr(0, index);

      value = 0;
      FromString(stringValue, value);
      rValue.SetY(value);

      // Get Z
      tempString = tempString.substr(index + 1, rStringValue.size());
      index = tempString.find_first_of(" ");

      stringValue = tempString.substr(index + 1, rStringValue.size());

      value = 0;
      FromString(stringValue, value);
      rValue.SetZ(value);

      return true;
    }

    return false;
  }

  // FromString for Pose2
  inline bool FromString(const std::string& rStringValue, Pose2& rValue)
  {
    size_t index = rStringValue.find_first_of(" ");
    if (index != std::string::npos)
    {
      std::stringstream converter;
      converter.str(rStringValue);

      double valueX = 0.0, valueY = 0.0, valueHeading = 0.0;
      converter >> valueX >> valueY >> valueHeading;

      rValue.SetX(valueX);
      rValue.SetY(valueY);
      rValue.SetHeading(valueHeading);
      return true;
    }
    return false;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  //@}

}

#endif // __OpenKarto_Geometry_h__
