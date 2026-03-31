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

#ifndef __OpenKarto_Meta_h__
#define __OpenKarto_Meta_h__

#include <MetaType.h>

namespace karto
{

  /**
   * Macro for adding a C++ class to the type system.
   */
  #define KARTO_TYPE(type) \
    template <> struct KartoTypeId<type> \
    { \
      static const char* Get(kt_bool = true) {return #type;} \
    }; \

  /**
   * Macro for getting the right runtime type info.
   */
  #define KARTO_RTTI() \
    public: virtual const char* GetKartoClassId() const {return GetKartoTypeIdTemplate(this);} \
    private:

}

#endif // __OpenKarto_Meta_h__
