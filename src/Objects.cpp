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

#include <Objects.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  ModuleParameters::ModuleParameters(const Identifier& rName)
    : Object(rName)
  {
  }

  ModuleParameters::~ModuleParameters()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  CustomItem::CustomItem()
    : Object()
  {
  }

  CustomItem::CustomItem(const Identifier& rName)
    : Object(rName)
  {
  }

  CustomItem::~CustomItem()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  DatasetInfo::DatasetInfo()
    : Object()
  {
    m_pTitle = new Parameter<std::string>(GetParameterSet(), "Title", "Dataset::Title", "Title of dataset", "");
    m_pAuthor = new Parameter<std::string>(GetParameterSet(), "Author", "Dataset::Author", "Author of dataset", "");
    m_pDescription = new Parameter<std::string>(GetParameterSet(), "Description", "Dataset::Description", "Description of dataset", "");
    m_pCopyright = new Parameter<std::string>(GetParameterSet(), "Copyright", "Dataset::Copyright", "Copyright of dataset", "");
  }

  DatasetInfo::~DatasetInfo()
  {
  }

  const std::string& DatasetInfo::GetTitle() const
  {
    return m_pTitle->GetValue();
  }

  const std::string& DatasetInfo::GetAuthor() const
  {
    return m_pAuthor->GetValue();
  }

  const std::string& DatasetInfo::GetDescription() const
  {
    return m_pDescription->GetValue();
  }

  const std::string& DatasetInfo::GetCopyright() const
  {
    return m_pCopyright->GetValue();
  }

}