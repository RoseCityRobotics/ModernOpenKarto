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

#include <map>
#include <iostream>
#include <algorithm>

#include <SensorRegistry.h>
#include <Exception.h>
#include <Sensor.h>
#include <Logger.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct SensorRegistryPrivate
  {
    std::vector<Sensor*> m_Sensors;

    typedef std::map<Identifier, Sensor*> SensorManagerMap;
    SensorManagerMap m_SensorMap;
  };

  SensorRegistry::SensorRegistry()
    : m_pSensorRegistryPrivate(new SensorRegistryPrivate())
  {
  }

  SensorRegistry::~SensorRegistry()
  {
    m_pSensorRegistryPrivate->m_Sensors.clear();
    delete m_pSensorRegistryPrivate;
  }

  SensorRegistry* SensorRegistry::GetInstance()
  {
    static SensorRegistry instance;
    return &instance;
  }

  void SensorRegistry::RegisterSensor(Sensor* pSensor)
  {
    if (pSensor != NULL)
    {
      if (pSensor->GetIdentifier().GetScope() != "Karto/System")
      {
        Validate(pSensor);

        Log(LOG_DEBUG, std::string("Registering sensor: [") + pSensor->GetIdentifier().ToString() + "]");
      }

      if ((m_pSensorRegistryPrivate->m_SensorMap.find(karto::Identifier(pSensor->GetIdentifier())) != m_pSensorRegistryPrivate->m_SensorMap.end()))
      {
        std::string errorMessage;
        errorMessage.append("Cannot register sensor: already registered: [");
        errorMessage.append(pSensor->GetIdentifier().ToString());
        errorMessage.append("]");

        throw Exception(errorMessage);
      }

      m_pSensorRegistryPrivate->m_SensorMap[karto::Identifier(pSensor->GetIdentifier())] = pSensor;
      m_pSensorRegistryPrivate->m_Sensors.push_back(pSensor);
    }
  }

  void SensorRegistry::UnregisterSensor(Sensor* pSensor)
  {
    if (pSensor != NULL)
    {
      if (pSensor->GetIdentifier().GetScope() != "Karto/System")
      {
        Log(LOG_DEBUG, std::string("Unregistering sensor: [") + pSensor->GetIdentifier().ToString() + "]");
      }

      if (m_pSensorRegistryPrivate->m_SensorMap.find(pSensor->GetIdentifier()) != m_pSensorRegistryPrivate->m_SensorMap.end())
      {
        m_pSensorRegistryPrivate->m_SensorMap.erase(pSensor->GetIdentifier());

        m_pSensorRegistryPrivate->m_Sensors.erase(std::remove(m_pSensorRegistryPrivate->m_Sensors.begin(), m_pSensorRegistryPrivate->m_Sensors.end(), pSensor), m_pSensorRegistryPrivate->m_Sensors.end());
      }
      else
      {
        std::string errorMessage;
        errorMessage.append("Cannot unregister sensor: not registered: [");
        errorMessage.append(pSensor->GetIdentifier().ToString());
        errorMessage.append("]");

        throw Exception(errorMessage);
      }
    }
  }

  Sensor* SensorRegistry::GetSensorByName(const Identifier& rName)
  {
    if (m_pSensorRegistryPrivate->m_SensorMap.find(rName) != m_pSensorRegistryPrivate->m_SensorMap.end())
    {
      Sensor* pSensor = m_pSensorRegistryPrivate->m_SensorMap[rName];

      assert(pSensor != NULL);

      return pSensor;
    }

    std::string errorMessage;
    errorMessage.append("Sensor not registered: [");
    errorMessage.append(rName.ToString());
    errorMessage.append("]");
    throw Exception(errorMessage);
  }

  void SensorRegistry::Clear()
  {
    m_pSensorRegistryPrivate->m_Sensors.clear();
    m_pSensorRegistryPrivate->m_SensorMap.clear();
  }

  void SensorRegistry::Validate(Sensor* pSensor)
  {
    if (pSensor == NULL)
    {
      throw Exception("Invalid sensor: NULL");
    }
    else if (pSensor->GetIdentifier().Size() == 0)
    {
      throw Exception("Invalid sensor: Nameless");        
    }
  }

}

