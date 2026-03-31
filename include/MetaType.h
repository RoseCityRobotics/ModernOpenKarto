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

#ifndef __OpenKarto_MetaType_h__
#define __OpenKarto_MetaType_h__

#include <type_traits>
#include <Types.h>

namespace karto
{

  /**
   * This will trigger a compile error if KARTO_TYPE is not defined for object
   */
  template <typename T>
  struct KartoTypeId
  {
    static const char* Get(kt_bool = true)
    {
      // Remember to register your class/enum T with the KARTO_TYPE macro
      return T::KARTO_REGISTER_ME_WITH_KARTO_TYPE();
    }
  };

  /**
   * Get composed type — strips const, reference, and pointer qualifiers.
   */
  template <typename T>
  struct KartoType
  {
    using Type = T;
  };

  template <typename T>
  struct KartoType<const T>
  {
    using Type = typename KartoType<T>::Type;
  };

  template <typename T>
  struct KartoType<T&>
  {
    using Type = typename KartoType<T>::Type;
  };

  template <typename T>
  struct KartoType<T*>
  {
    using Type = typename KartoType<T>::Type;
  };

  /**
   * Get the Karto type id
   */
  template <typename T>
  const char* GetKartoTypeIdTemplate()
  {
    return KartoTypeId<typename KartoType<T>::Type>::Get();
  }

  template <typename T>
  const char* GetKartoTypeIdTemplate(const T&)
  {
    return KartoTypeId<typename KartoType<T>::Type>::Get();
  }

  /**
   * Karto object traits — generic version
   */
  template <typename T, typename E = void>
  struct KartoObjectTraits
  {
    using RefReturnType = T&;

    static RefReturnType Get(void* pPointer)
    {
      return *static_cast<T*>(pPointer);
    }
  };

  template <typename T>
  struct KartoObjectTraits<T*>
  {
    using RefReturnType = T*;
    using PointerType = T*;

    static RefReturnType Get(void* pPointer)
    {
      return static_cast<T*>(pPointer);
    }

    static PointerType GetPointer(T* rValue)
    {
      return rValue;
    }
  };

  template <typename T>
  struct KartoObjectTraits<T&, std::enable_if_t<!std::is_pointer_v<typename KartoObjectTraits<T>::RefReturnType>>>
  {
    using RefReturnType = T&;
    using PointerType = T*;

    static RefReturnType Get(void* pPointer)
    {
      return *static_cast<T*>(pPointer);
    }

    static PointerType GetPointer(T& rValue)
    {
      return &rValue;
    }
  };

  template <typename T>
  struct KartoObjectTraits<T&, std::enable_if_t<std::is_pointer_v<typename KartoObjectTraits<T>::RefReturnType>>> : KartoObjectTraits<T>
  {
  };

  template <typename T>
  struct KartoObjectTraits<const T> : KartoObjectTraits<T>
  {
  };

  /**
   * Compile time check if T implements the KARTO_RTTI (Karto Runtime Type Info)
   */
  template <typename T>
  struct KartoTypeHasRtti
  {
    using Yes = kt_int16s;
    using No = kt_int32s;

    template <typename U, const char* (U::*)() const>
    struct CheckForMember
    {
    };

    template <typename U> static Yes HasMemberFunction(CheckForMember<U, &U::GetKartoClassId>*);
    template <typename U> static No HasMemberFunction(...);

    enum
    {
      value = sizeof(HasMemberFunction<typename KartoType<T>::Type>(0)) == sizeof(Yes)
    };
  };

  /**
   * Get KARTO_RTTI information for dynamic karto type.
   */
  template <typename T, typename E = void>
  struct GetKartoTypeIdTemplateRTTI
  {
    using Traits = KartoObjectTraits<const T&>;

    static const char* Get(const T& rObject)
    {
      typename Traits::PointerType pointer = Traits::GetPointer(rObject);
      return pointer ? pointer->GetKartoClassId() : GetKartoTypeIdTemplate<T>();
    }
  };

  template <typename T>
  struct GetKartoTypeIdTemplateRTTI<T, std::enable_if_t<!KartoTypeHasRtti<T>::value>>
  {
    static const char* Get(const T&)
    {
      return GetKartoTypeIdTemplate<T>();
    }
  };

  /**
   * Get the Karto type id
   */
  template <typename T> const char* GetTypeId()
  {
    return GetKartoTypeIdTemplate<T>();
  }

  template <typename T> const char* GetTypeId(const T& rObject)
  {
    return GetKartoTypeIdTemplateRTTI<T>::Get(rObject);
  }

}

#endif // __OpenKarto_MetaType_h__
