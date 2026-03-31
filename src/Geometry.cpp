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

#include <Geometry.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  Pose2::Pose2()
    : m_Heading(0.0)
  {
  }

  Pose2::Pose2(const Vector2d& rPosition, double heading)
    : m_Position(rPosition)
    , m_Heading(heading)
  {
  }

  Pose2::Pose2(double x, double y, double heading)
    : m_Position(x, y)
    , m_Heading(heading)
  {
  }

  Pose2::Pose2(const Pose2& rOther)
    : m_Position(rOther.m_Position)
    , m_Heading(rOther.m_Heading)
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  BoundingBox2::BoundingBox2()
    : m_Minimum(DBL_MAX, DBL_MAX)
    , m_Maximum(-DBL_MAX, -DBL_MAX)
  {
  }

  BoundingBox2::BoundingBox2(const Vector2d& rMinimum, const Vector2d& rMaximum)
    : m_Minimum(rMinimum)
    , m_Maximum(rMaximum)
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

}