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

#include <algorithm>
#include <string>
#include <limits>

#include <StringHelper.h>
#include <Geometry.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  std::string StringHelper::ToString(const char* value)
  {
    return std::string(value);
  }

  std::string StringHelper::ToString(kt_bool value)
  {
    if (value == true)
    {
      return std::string("true");
    }

    return std::string("false");
  }

  std::string StringHelper::ToString(kt_int16u value)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter << value;
    return converter.str();
  }

  std::string StringHelper::ToString(kt_int16s value)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter << value;
    return converter.str();
  }

  std::string StringHelper::ToString(kt_int32u value)
  {
    char buffer[64];
#ifdef WIN32
    sprintf_s(buffer, 64, "%u", value);
#else
    sprintf(buffer, "%u", value);
#endif
    return std::string(buffer);
  }

  std::string StringHelper::ToString(kt_int32s value)
  {
    char buffer[64];
#ifdef WIN32
    sprintf_s(buffer, 64, "%d", value);
#else
    sprintf(buffer, "%d", value);
#endif
    return std::string(buffer);
  }

  std::string StringHelper::ToString(kt_int64u value)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter << value;
    return converter.str();
  }

  std::string StringHelper::ToString(kt_int64s value)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter << value;
    return converter.str();
  }

#if !KARTO_SIZE_T_SAME_AS_UINT64
  std::string StringHelper::ToString(kt_size_t value)
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);
    converter << value;
    return converter.str();
  }
#endif

  std::string StringHelper::ToString(kt_float value)
  {
    char buffer[64];
#ifdef WIN32
    sprintf_s(buffer, 64, "%.*g", 8, (double) value);
#else
    sprintf(buffer, "%.*g", 8, (double) value);
#endif
    return std::string(buffer);
  }

  std::string StringHelper::ToString(kt_double value)
  {
    char buffer[64];
#ifdef WIN32
    sprintf_s(buffer, 64, "%.*g", 16, value);
#else
    sprintf(buffer, "%.*g", 16, value);
#endif
    return std::string(buffer);
  }

  std::string StringHelper::ToString(kt_float value, kt_int32u precision)
  {
    char buffer[64];
#ifdef WIN32
    sprintf_s(buffer, 64, "%.*f", (kt_int32s)precision, (double)value);
#else
    sprintf(buffer, "%.*f", (kt_int32s)precision, (double)value);
#endif
    return std::string(buffer);
  }

  std::string StringHelper::ToString(kt_double value, kt_int32u precision)
  {
    char buffer[64];
#ifdef WIN32
    sprintf_s(buffer, 64, "%.*f", (kt_int32s)precision, value);
#else
    sprintf(buffer, "%.*f", (kt_int32s)precision, value);
#endif
    return std::string(buffer);
  }

  std::string StringHelper::ToString(const std::string& rValue)
  {
    return rValue;
  }

  std::string StringHelper::ToString(const Quaternion& rValue)
  {
    return rValue.ToString();
  }

  std::string StringHelper::ToString(const Color& rValue)
  {
    return rValue.ToString();
  }

  std::string StringHelper::ToString(const Pose2& rValue)
  {
    return rValue.ToString();
  }

  std::string StringHelper::ToString(const Pose3& rValue)
  {
    return rValue.ToString();
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, kt_bool& rValue)
  {
    rValue = false;

    if (ToLowerCase(rStringValue) == std::string("true"))
    {
      rValue = true;
    }

    return true;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, kt_int16s& rValue)
  {
    int precision = std::numeric_limits<double>::digits10;
    std::stringstream converter;
    converter.precision(precision);

    converter.str(rStringValue);

    converter >> rValue;

    return true;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, kt_int16u& rValue)
  {
    int precision = std::numeric_limits<double>::digits10;
    std::stringstream converter;
    converter.precision(precision);

    converter.str(rStringValue);

    converter >> rValue;

    return true;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, kt_int32s& rValue)
  {
    int precision = std::numeric_limits<double>::digits10;
    std::stringstream converter;
    converter.precision(precision);

    converter.str(rStringValue);

    converter >> rValue;

    return true;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, kt_int32u& rValue)
  {
    int precision = std::numeric_limits<double>::digits10;
    std::stringstream converter;
    converter.precision(precision);

    converter.str(rStringValue);

    converter >> rValue;

    return true;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, kt_int64s& rValue)
  {
    int precision = std::numeric_limits<double>::digits10;
    std::stringstream converter;
    converter.precision(precision);

    converter.str(rStringValue);

    converter >> rValue;

    return true;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, kt_int64u& rValue)
  {
    int precision = std::numeric_limits<double>::digits10;
    std::stringstream converter;
    converter.precision(precision);

    converter.str(rStringValue);

    converter >> rValue;

    return true;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, kt_float& rValue)
  {
    int precision = std::numeric_limits<double>::digits10;
    std::stringstream converter;
    converter.precision(precision);

    converter.str(rStringValue);

    converter >> rValue;

    return true;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, kt_double& rValue)
  {
    int precision = std::numeric_limits<double>::digits10;
    std::stringstream converter;
    converter.precision(precision);

    converter.str(rStringValue);

    converter >> rValue;

    return true;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, std::string& rValue)
  {
    rValue = rStringValue;

    return true;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, Quaternion& rValue)
  {
    kt_size_t index = rStringValue.find_first_of(" ");
    if (index != std::string::npos)
    {
      std::stringstream converter;
      converter.str(rStringValue);

      kt_double valueX = 0.0;
      kt_double valueY = 0.0;
      kt_double valueZ = 0.0;
      kt_double valueW = 0.0;

      converter >> valueX;
      converter >> valueY;
      converter >> valueZ;
      converter >> valueW;

      rValue.SetX(valueX);
      rValue.SetY(valueY);
      rValue.SetZ(valueZ);
      rValue.SetW(valueW);

      return true;
    }

    return false;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, Color& rValue)
  {
    kt_size_t index = rStringValue.find_first_of(" ");
    if (index != std::string::npos)
    {
      std::stringstream converter;
      converter.str(rStringValue);

      kt_double valueRed = 0.0;
      kt_double valueGreen = 0.0;
      kt_double valueBlue = 0.0;
      kt_double valueAlpha = 0.0;

      converter >> valueRed;
      converter >> valueGreen;
      converter >> valueBlue;
      converter >> valueAlpha;

      rValue.SetRed(valueRed);
      rValue.SetGreen(valueGreen);
      rValue.SetBlue(valueBlue);
      rValue.SetAlpha(valueAlpha);

      return true;
    }

    return false;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, Pose2& rValue)
  {
    kt_size_t index = rStringValue.find_first_of(" ");
    if (index != std::string::npos)
    {
      std::stringstream converter;
      converter.str(rStringValue);

      kt_double valueX = 0.0;
      kt_double valueY = 0.0;
      kt_double valueHeading = 0.0;

      converter >> valueX;
      converter >> valueY;
      converter >> valueHeading;

      rValue.SetX(valueX);
      rValue.SetY(valueY);
      rValue.SetHeading(valueHeading);

      return true;
    }

    return false;
  }

  kt_bool StringHelper::FromString(const std::string& rStringValue, Pose3& rValue)
  {
    kt_size_t index = rStringValue.find_first_of(" ");
    if (index != std::string::npos)
    {
      std::stringstream converter;
      converter.str(rStringValue);

      kt_double valueX = 0.0;
      kt_double valueY = 0.0;
      kt_double valueZ = 0.0;
      kt_double valueW = 0.0;

      converter >> valueX;
      converter >> valueY;
      converter >> valueZ;
      rValue.SetPosition(karto::Vector3d(valueX, valueY, valueZ));

      valueX = 0.0;
      valueY = 0.0;
      valueZ = 0.0;
      valueW = 0.0;

      converter >> valueX;
      converter >> valueY;
      converter >> valueZ;
      converter >> valueW;
      rValue.SetOrientation(karto::Quaternion(valueX, valueY, valueZ, valueW));

      return true;
    }

    return false;
  }

  std::string StringHelper::Trim(const std::string& rValue)
  {
    char const* delims = " \t\r\n";

    std::string result(rValue);
    std::string::size_type index = result.find_last_not_of(delims);
    if (index != std::string::npos)
      result.erase(++index);

    index = result.find_first_not_of(delims);
    if (index != std::string::npos)
    {
      result.erase(0, index);
    }
    else
    {
      result.erase();
    }

    return result;
  }

  std::string StringHelper::Replace(const std::string& rSource, const std::string& rFind, const std::string& rReplace)
  {
    size_t j;

    std::string retStr = rSource;

    if (rFind == rReplace)
    {
      return retStr;
    }

    for (; ( j = retStr.find(rFind) ) != std::string::npos; )
    {
      retStr.replace( j, rFind.size(), rReplace);
    }

    return retStr;
  }

  kt_bool StringHelper::IsLetter(char ch)
  {
    return isalpha(ch) != 0;
  }

  std::string StringHelper::ToLowerCase(const std::string &rValue)
  {
    std::string result = rValue;

    std::transform(rValue.begin(), rValue.end(), result.begin(), tolower);

    return result;
  }

  std::string StringHelper::ToUpperCase(const std::string &rValue)
  {
    std::string result = rValue;

    std::transform(rValue.begin(), rValue.end(), result.begin(), toupper);

    return result;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  void StringBuilder::Clear()
  {
    m_String = "";
  }

  const std::string& StringBuilder::ToString() const
  {
    return m_String;
  }

  //StringBuilder& StringBuilder::operator << (char value)
  //{
  //  m_String.append(value);

  //  return *this;
  //}

  StringBuilder& StringBuilder::operator << (kt_int8u value)
  {
    m_String.append(karto::StringHelper::ToString(value));

    return *this;
  }

  StringBuilder& StringBuilder::operator << (kt_int16s value)
  {
    m_String.append(karto::StringHelper::ToString(value));

    return *this;
  }

  StringBuilder& StringBuilder::operator << (kt_int16u value)
  {
    m_String.append(karto::StringHelper::ToString(value));

    return *this;
  }

  StringBuilder& StringBuilder::operator << (kt_int32s value)
  {
    m_String.append(karto::StringHelper::ToString(value));

    return *this;
  }

  StringBuilder& StringBuilder::operator << (kt_int32u value)
  {
    m_String.append(karto::StringHelper::ToString(value));

    return *this;
  }

  StringBuilder& StringBuilder::operator << (kt_int64s value)
  {
    m_String.append(karto::StringHelper::ToString(value));

    return *this;
  }

  StringBuilder& StringBuilder::operator << (kt_int64u value)
  {
    m_String.append(karto::StringHelper::ToString(value));

    return *this;
  }

#if !KARTO_SIZE_T_SAME_AS_UINT64
  StringBuilder& StringBuilder::operator << (kt_size_t value)
  {
    m_String.append(karto::StringHelper::ToString(value));

    return *this;
  }
#endif

  StringBuilder& StringBuilder::operator << (kt_float value)
  {
    m_String.append(karto::StringHelper::ToString(value));

    return *this;
  }

  StringBuilder& StringBuilder::operator << (kt_double value)
  {
    m_String.append(karto::StringHelper::ToString(value));

    return *this;
  }

	StringBuilder& StringBuilder::operator << (const std::string& rValue)
  {
    m_String.append(rValue);

    return *this;
  }

  StringBuilder& StringBuilder::operator << (const StringBuilder& rValue)
  {
    m_String.append(rValue.ToString());

    return *this;
  }

}
