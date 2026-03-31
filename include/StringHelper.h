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

#ifndef __OpenKarto_StringHelper_h__
#define __OpenKarto_StringHelper_h__

#include <stdio.h>
#include <ostream>
#include <sstream>
#include <string>

#include <Macros.h>
#include <Types.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class Color;

  class Pose2;
  class Pose3;
  class Quaternion;

  template<typename T> class Size2;

  template<typename T> class Vector2;
  template<typename T> class Vector3;
  template<typename T> class Vector4;

  /**
   * Class to convert to and from Karto strings
   */
  class KARTO_EXPORT StringHelper
  {
  public:
    /**
     * Converts the given C-string to a string
     * @param value value to be converted
     * @return string representation of the C-string
     */
    static std::string ToString(const char* value);

    /**
     * Converts the given boolean to a string
     * @param value value to be converted
     * @return string representation of the boolean
     */
    static std::string ToString(kt_bool value);

    /**
     * Converts the given 16-bit signed integer to a string
     * @param value value to be converted
     * @return string representation of the 16-bit signed integer
     */
    static std::string ToString(kt_int16s value);

    /**
     * Converts the given 16-bit unsigned integer to a string
     * @param value value to be converted
     * @return string representation of the 16-bit unsigned integer
     */
    static std::string ToString(kt_int16u value);

    /**
     * Converts the given 32-bit signed integer to a string
     * @param value value to be converted
     * @return string representation of the 32-bit signed integer
     */
    static std::string ToString(kt_int32s value);

    /**
     * Converts the given 32-bit unsigned integer to a string
     * @param value value to be converted
     * @return string representation of the 32-bit unsigned integer
     */
    static std::string ToString(kt_int32u value);

    /**
     * Converts the given 64-bit signed integer to a string
     * @param value value to be converted
     * @return string representation of the 64-bit signed integer
     */
    static std::string ToString(kt_int64s value);

    /**
     * Converts the given 64-bit unsigned integer to a string
     * @param value value to be converted
     * @return string representation of the 64-bit unsigned integer
     */
    static std::string ToString(kt_int64u value);

#if !KARTO_SIZE_T_SAME_AS_UINT64
    // size_t is a distinct type from uint64_t on macOS (unsigned long vs unsigned long long)
    static std::string ToString(kt_size_t value);
#endif

    /**
     * Converts the given float to a string
     * @param value value to be converted
     * @return string representation of the float
     */
    static std::string ToString(kt_float value);

    /**
     * Converts the given double to a string
     * @param value value to be converted
     * @return string representation of the double
     */
    static std::string ToString(kt_double value);

    /**
     * Converts the given float to a string with the given precision
     * @param value value to be converted
     * @param precision precision
     * @return string representation of the float
     */
    static std::string ToString(kt_float value, kt_int32u precision);

    /**
     * Converts the given double to a string with the given precision
     * @param value value to be converted
     * @param precision precision
     * @return string representation of the double
     */
    static std::string ToString(kt_double value, kt_int32u precision);

    /**
     * Copies the string argument
     * @param rValue string value
     * @return copy of the string argument
     */
    static std::string ToString(const std::string& rValue);

    /**
     * Converts the given size to a string
     * @param rValue value to be converted
     * @return string representation of the size
     */
    template<typename T>
    inline static std::string ToString(const Size2<T>& rValue);

    /**
     * Converts the given vector to a string
     * @param rValue value to be converted
     * @return string representation of the vector
     */
    template<typename T>
    inline static std::string ToString(const Vector2<T>& rValue);

    /**
     * Converts the given vector to a string
     * @param rValue value to be converted
     * @return string representation of the vector
     */
    template<typename T>
    inline static std::string ToString(const Vector3<T>& rValue);

    /**
     * Converts the given vector to a string
     * @param rValue value to be converted
     * @return string representation of the vector
     */
    template<typename T>
    inline static std::string ToString(const Vector4<T>& rValue);

    /**
     * Converts the given quaternion to a string
     * @param rValue value to be converted
     * @return string representation of the quaternion
     */
    static std::string ToString(const Quaternion& rValue);

    /**
     * Converts the given color to a string
     * @param rValue value to be converted
     * @return string representation of the color
     */
    static std::string ToString(const Color& rValue);

    /**
     * Converts the given pose to a string
     * @param rValue value to be converted
     * @return string representation of the pose
     */
    static std::string ToString(const Pose2& rValue);

    /**
     * Converts the given pose to a string
     * @param rValue value to be converted
     * @return string representation of the pose
     */
    static std::string ToString(const Pose3& rValue);

    /**
     * Converts the given string to a boolean
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, kt_bool& rValue);

    /**
     * Converts the given string to a 16-bit signed integer
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, kt_int16s& rValue);

    /**
     * Converts the given string to a 16-bit unsigned integer
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, kt_int16u& rValue);

    /**
     * Converts the given string to a 32-bit signed integer
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, kt_int32s& rValue);

    /**
     * Converts the given string to a 32-bit unsigned integer
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, kt_int32u& rValue);

    /**
     * Converts the given string to a 64-bit signed integer
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, kt_int64s& rValue);

    /**
     * Converts the given string to a 64-bit unsigned integer
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, kt_int64u& rValue);

    /**
     * Converts the given string to a float
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, kt_float& rValue);

    /**
     * Converts the given string to a double
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, kt_double& rValue);

    /**
     * Converts the given std::string to a std::string
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, std::string& rValue);

    /**
     * Converts the given std::string to a Size2<T>
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    template<typename T>
    static kt_bool FromString(const std::string& rStringValue, Size2<T>& rValue)
    {
      kt_size_t index = rStringValue.find_first_of(" ");
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

    /**
     * Converts the given std::string to a Vector2<T>
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    template<typename T>
    static kt_bool FromString(const std::string& rStringValue, Vector2<T>& rValue)
    {
      kt_size_t index = rStringValue.find_first_of(" ");
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

    /**
     * Converts the given std::string to a Vector3<T>
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    template<typename T>
    inline static kt_bool FromString(const std::string& rStringValue, Vector3<T>& rValue)
    {
      std::string tempString = rStringValue;
      kt_size_t index = tempString.find_first_of(" ");
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

    /**
     * Converts the given std::string to a Vector4<T>
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    template<typename T>
    static kt_bool FromString(const std::string& rStringValue, Vector4<T>& rValue)
    {
      std::string tempString = rStringValue;
      kt_size_t index = tempString.find_first_of(" ");
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

        // Get W
        tempString = tempString.substr(index + 1, rStringValue.size());
        index = tempString.find_first_of(" ");

        stringValue = tempString.substr(index + 1, rStringValue.size());

        value = 0;
        FromString(stringValue, value);
        rValue.SetW(value);

        return true;
      }

      return false;
    }

    /**
     * Converts the given std::string to a Quaternion
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, Quaternion& rValue);

    /**
     * Converts the given std::string to a Color
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, Color& rValue);

    /**
     * Converts the given std::string to a Pose2
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, Pose2& rValue);

    /**
     * Converts the given std::string to a Pose3
     * @param rStringValue string representation of value
     * @param rValue value to set from string
     * @return true if conversion was success, false otherwise
     */
    static kt_bool FromString(const std::string& rStringValue, Pose3& rValue);

    /**
     * Returns a trimmed version of the given string
     * @param rValue string
     * @return trimmed version of the given string
     */
    static std::string Trim(const std::string& rValue);

    /**
     * Replace all instances of string pattern in source string with replacement pattern
     * @param rSource source string
     * @param rFind string pattern
     * @param rReplace replacement pattern
     * @return replaced string
     */
    static std::string Replace(const std::string& rSource, const std::string& rFind, const std::string& rReplace);

    /**
     * Checks if given character is a letter
     * @param ch character
     * @return true if given character is a letter
     */
    static kt_bool IsLetter(char ch);

    /**
     * Returns a lowercase version of the given string
     * @param rValue string
     * @return lowercase version of the given string
     */
    static std::string ToLowerCase(const std::string &rValue);

    /**
     * Returns a uppercase version of the given string
     * @param rValue string
     * @return uppercase version of the given string
     */
    static std::string ToUpperCase(const std::string &rValue);
  }; // class StringHelper

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Class to build strings
   */
  class KARTO_EXPORT StringBuilder
  {
  public:
    /**
     * Convert contents to string
     * @return string of contents
     */
    const std::string& ToString() const;

    /**
     * Erase contents of string builder
     */
    void Clear();

  public:
    /**
     * Add char to this string builder
     * @param value char value
     */
//    StringBuilder& operator << (char value);

    /**
     * Add kt_int8u to this string builder
     * @param value kt_int8u value
     * @return this StringBuilder with given value added
     */
    StringBuilder& operator << (kt_int8u value);

    /**
     * Add kt_int16s to this string builder
     * @param value kt_int16s value
     * @return this StringBuilder with given value added
     */
    StringBuilder& operator << (kt_int16s value);
    /**
     * Add kt_int16u to this string builder
     * @param value kt_int16u value
     * @return this StringBuilder with given value added
     */
    StringBuilder& operator << (kt_int16u value);

    /**
     * Add kt_int32s to this string builder
     * @param value kt_int32s value
     * @return this StringBuilder with given value added
     */
    StringBuilder& operator << (kt_int32s value);
    /**
     * Add kt_int32u to this string builder
     * @param value kt_int32u value
     * @return this StringBuilder with given value added
     */
    StringBuilder& operator << (kt_int32u value);

    /**
     * Add kt_int64s to this string builder
     * @param value kt_int64s value
     * @return this StringBuilder with given value added
     */
    StringBuilder& operator << (kt_int64s value);
    /**
     * Add kt_int64u to this string builder
     * @param value kt_int64u value
     * @return this StringBuilder with given value added
     */
    StringBuilder& operator << (kt_int64u value);

#if !KARTO_SIZE_T_SAME_AS_UINT64
    // size_t is a distinct type from uint64_t on macOS
    StringBuilder& operator << (kt_size_t value);
#endif

    /**
     * Add kt_float to this string builder
     * @param value kt_float value
     * @return this StringBuilder with given value added
     */
    StringBuilder& operator << (kt_float value);
    /**
     * Add kt_double to this string builder
     * @param value kt_double value
     * @return this StringBuilder with given value added
     */
    StringBuilder& operator << (kt_double value);

    /**
     * Add string to this string builder
     * @param rValue string value
     * @return this StringBuilder with given value added
     */
    StringBuilder& operator << (const std::string& rValue);

    /**
     * Add string builder to this string builder
     * @param rValue string builder value
     * @return this StringBuilder with given value added
     */
    StringBuilder& operator << (const StringBuilder& rValue);

    /**
     * Write string to output stream
     * @param rStream output stream
     * @param rStringBuilder StringBuilder to write
     * @return rStream
     */
    friend KARTO_FORCEINLINE std::ostream& operator << (std::ostream& rStream, const StringBuilder& rStringBuilder)
    {
      rStream << rStringBuilder.ToString();
      return rStream;
    }

  private:
    std::string m_String;
  }; // class StringBuilder

  //@}

}

#endif // __OpenKarto_StringHelper_h__
