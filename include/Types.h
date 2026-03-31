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

#ifndef __OpenKarto_Types_h__
#define __OpenKarto_Types_h__

#include <cassert>
#include <cstddef>
#include <cstdint>

using kt_int8s  = std::int8_t;
using kt_int8u  = std::uint8_t;
using kt_int16s = std::int16_t;
using kt_int16u = std::uint16_t;
using kt_int32s = std::int32_t;
using kt_int32u = std::uint32_t;
using kt_int64s = std::int64_t;
using kt_int64u = std::uint64_t;

using kt_size_t = std::size_t;

// On ARM64 Linux (Jetson, etc.), size_t and uint64_t are both 'unsigned long',
// making them the same type. On macOS and x86_64 Linux, they differ.
#if defined(__linux__) && defined(__aarch64__)
#define KARTO_SIZE_T_SAME_AS_UINT64 1
#else
#define KARTO_SIZE_T_SAME_AS_UINT64 0
#endif

using kt_bool       = bool;
using kt_char        = char;
using kt_float       = float;
using kt_double      = double;
using kt_objecttype  = kt_int32u;
using kt_tick        = kt_int64s;

#endif // __OpenKarto_Types_h__
