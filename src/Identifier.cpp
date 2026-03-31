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

#include <Identifier.h>
#include <Exception.h>

namespace karto
{

  Identifier::Identifier()
  {
  }

  Identifier::Identifier(const char* pString)
  {
    Parse(pString);
  }

  Identifier::Identifier(const std::string& rString)
  {
    Parse(rString);
  }

  Identifier::Identifier(const Identifier& rOther)
  {
    Parse(rOther.ToString());
  }

  Identifier::~Identifier()
  {
  }

  const std::string& Identifier::GetName() const
  {
    return m_Name;
  }

  void Identifier::SetName(const std::string& rName)
  {
    if (rName.size() != 0)
    {
      std::string::size_type pos = rName.find_last_of('/');
      if (pos != 0 && pos != std::string::npos)
      {
        throw Exception("Name can't contain a scope!");
      }

      m_Name = rName;
    }
    else
    {
      m_Name.clear();
    }

    Update();
  }

  const std::string& Identifier::GetScope() const
  {
    return m_Scope;
  }

  void Identifier::SetScope(const std::string& rScope)
  {
    if (rScope.size() != 0)
    {
      m_Scope = rScope;
    }
    else
    {
      m_Scope.clear();
    }

    Update();
  }

  const std::string& Identifier::ToString() const
  {
    return m_FullName;
  }

  void Identifier::Clear()
  {
    m_Name.clear();
    m_Scope.clear();
    m_FullName.clear();
  }

  void Identifier::Parse(const std::string& rString)
  {
    if (rString.size() == 0)
    {
      Clear();
      return;
    }

    std::string::size_type pos = rString.find_last_of('/');

    if (pos == std::string::npos)
    {
      m_Name = rString;
    }
    else
    {
      m_Scope = rString.substr(0, pos);
      m_Name = rString.substr(pos+1, rString.size());

      // remove '/' from m_Scope if first!!
      if (m_Scope.size() > 0 && m_Scope[0] == '/')
      {
        m_Scope = m_Scope.substr(1, m_Scope.size());
      }
    }

    Update();
  }

  void Identifier::Validate(const std::string& rString)
  {
    if (rString.size() == 0)
    {
      return;
    }

    char c = rString[0];
    if (IsValidFirst(c))
    {
      for (size_t i = 1; i < rString.size(); ++i)
      {
        c = rString[i];
        if (!IsValid(c))
        {
          throw Exception("Invalid character in name. Valid characters must be within the ranges A-Z, a-z, 0-9, '/', '_' and '-'.");
        }
      }
    }
    else
    {
      throw Exception("Invalid first character in name. Valid characters must be within the ranges A-Z, a-z, and '/'.");
    }
  }

  void Identifier::Update()
  {
    m_FullName.clear();

    if (m_Scope.size() > 0)
    {
      m_FullName.append("/");
      m_FullName.append(m_Scope);
      m_FullName.append("/");
    }
    m_FullName.append(m_Name);
  }

  Identifier& Identifier::operator=(const Identifier& rOther)
  {
    if (&rOther != this)
    {
      m_Name = rOther.m_Name;
      m_Scope = rOther.m_Scope;
      m_FullName = rOther.m_FullName;
    }

    return *this;
  }

  kt_bool Identifier::operator==(const Identifier& rOther) const
  {
    return (m_FullName == rOther.m_FullName);
  }

  kt_bool Identifier::operator<(const Identifier& rOther) const
  {
    return m_FullName < rOther.m_FullName;
  }

  kt_size_t Identifier::Size() const
  {
    return m_FullName.size();
  }

}
