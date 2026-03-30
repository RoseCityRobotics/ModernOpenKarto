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

#ifndef __OpenKarto_h__
#define __OpenKarto_h__

#include <iostream>

#include <AbstractGpsEstimationManager.h>
#include <CoordinateConverter.h>
#include <Deprecated.h>
#include <Event.h>
#include <Exception.h>
#include <Geometry.h>
#include <Grid.h>
#include <GridIndexLookup.h>
#include <Identifier.h>
#include <List.h>
#include <Logger.h>
#include <Macros.h>
#include <KartoMath.h>
#include <Meta.h>
#include <MetaClassHelper.h>
#include <MetaEnumHelper.h>
#include <Module.h>
#include <Object.h>
#include <Objects.h>
#include <OccupancyGrid.h>
#include <OpenMapper.h>
#include <Pair.h>
#include <Parameter.h>
#include <PoseTransform.h>
#include <RangeTransform.h>
#include <Referenced.h>
#include <RigidBodyTransform.h>
#include <Sensor.h>
#include <SensorData.h>
#include <SensorRegistry.h>
#include <SmartPointer.h>
#include <KartoString.h>
#include <StringHelper.h>
#include <TypeCasts.h>
#include <Types.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  //@cond EXCLUDE
  /**
   * Internal function please don't call. 
   * Initialize and register OpenKarto classes with MetaClassManager
   * @note Please don't call. Called by Environment::Initialize()
   */
  void InitializeOpenKartoMetaClasses();
  //@endcond

  //@}

};

#endif // __OpenKarto_h__
