# Migration Guide: OpenKarto to ModernOpenKarto

This document lists all breaking API changes from the original `skasperski/OpenKarto` to this modernized C++17 fork.

## Build System

- **CMake 3.16+** required (was 2.6)
- **C++17** required (`CMAKE_CXX_STANDARD 17`)
- **No external dependencies** — TBB has been completely eliminated
- Google Test fetched automatically for tests (`OPENKARTO_BUILD_TESTS=ON`)
- Use via `add_subdirectory()` or `FetchContent`; exports `OpenKarto::OpenKarto` target

## Directory Layout

| Before | After |
|--------|-------|
| `source/*.cpp` | `src/*.cpp` |
| `source/OpenKarto/*.h` | `include/*.h` |
| (none) | `test/*.cpp` |

## Include Path Changes

All `#include <OpenKarto/Foo.h>` become `#include <Foo.h>`.

Two headers were renamed to avoid case-insensitive filesystem collisions:
- `String.h` -> `KartoString.h` (then deleted — see below)
- `Math.h` -> `KartoMath.h`

## Removed Headers

| Header | Reason |
|--------|--------|
| `String.h` / `KartoString.h` | `karto::String` replaced by `std::string` |
| `List.h` | `karto::List<T>` replaced by `std::vector<T>` |
| `Pair.h` | `karto::Pair<K,V>` replaced by `std::pair<K,V>` |
| `Any.h` | `karto::Any` replaced by `std::any` |
| `SmartPointer.h` | `karto::SmartPointer<T>` replaced by `std::shared_ptr<T>` |
| `Referenced.h` | Intrusive ref-counting base class removed |
| `Mutex.h` | `karto::Mutex` replaced by `std::mutex` |
| `Deprecated.h` | Contained only obsolete typedefs |
| `MetaClass.h`, `MetaClassHelper.h`, `MetaClassManager.h` | Dead meta/reflection system removed |
| `MetaEnum.h`, `MetaEnumHelper.h`, `MetaEnumManager.h` | Dead meta/reflection system removed |
| `MetaAttribute.h` | Dead meta/reflection system removed |

## Type Replacements

| Before | After |
|--------|-------|
| `karto::String` | `std::string` |
| `karto::List<T>` | `std::vector<T>` |
| `karto::Pair<K,V>` | `std::pair<K,V>` |
| `karto::Any` | `std::any` |
| `karto::SmartPointer<T>` | `std::shared_ptr<T>` |
| `karto::Mutex` | `std::mutex` |
| `karto::Mutex::ScopedLock` | `std::lock_guard<std::mutex>` |
| `NULL` | `nullptr` |

## Method Replacements

### String (now std::string)
| Before | After |
|--------|-------|
| `.ToCString()` | `.c_str()` |
| `.Append(x)` | `+= x` or `.append(x)` |
| `.Size()` | `.size()` |
| `.SubString(i, n)` | `.substr(i, n)` |
| `.Find(x)` | `.find(x)` |
| `.FindFirstOf(x)` | `.find_first_of(x)` |
| `.FindLastOf(x)` | `.find_last_of(x)` |
| `String::NewLine()` | `"\n"` |

### List (now std::vector)
| Before | After |
|--------|-------|
| `.Add(x)` | `.push_back(x)` |
| `.Remove(x)` | `.erase(std::remove(...), .end())` |
| `.Contains(x)` | `std::find(...) != .end()` |
| `.Size()` | `.size()` |
| `.IsEmpty()` | `.empty()` |
| `.BinarySearch(v, f)` | `std::lower_bound(...)` |
| `karto_forEach(Type, &list)` | `for (auto& item : list)` |

### Pair (now std::pair)
| Before | After |
|--------|-------|
| `.GetFirst()` | `.first` |
| `.GetSecond()` | `.second` |

## Smart Pointer Changes

`karto::SmartPointer<T>` used intrusive reference counting via the `Referenced` base class. It has been replaced by `std::shared_ptr<T>`.

**Key differences:**
- `SmartPointer<T>` had implicit conversion from `T*`. `std::shared_ptr<T>` does not. Use `std::make_shared<T>(...)` or `std::shared_ptr<T>(ptr)`.
- The `Referenced` base class no longer exists. Classes no longer inherit from it.
- Protected destructors are now public virtual.
- Ptr typedefs (`ObjectPtr`, `SensorPtr`, etc.) are now `std::shared_ptr<T>` aliases.
- Non-owning lists (e.g., `LocalizedLaserScanList`) use raw pointers, not shared_ptr.

## SensorRegistry Changes

The global `SensorRegistry` singleton still exists for backward compatibility, but scan data classes now support a direct `LaserRangeFinder*` pointer:

```cpp
scan->SetLaserRangeFinder(laser);  // Bypasses global registry
laser = scan->GetLaserRangeFinder(); // Checks direct pointer first, then registry
```

This allows multiple independent Mapper instances with the same sensor name.

## Removed APIs

- `InitializeOpenKartoMetaClasses()` — removed (was never called)
- `GetMetaClassByName()`, `GetMetaClassByType()`, etc. — removed
- `MetaEnum::Register()` — removed; use `ParameterEnum::DefineEnumValue()` directly
- `KARTO_AUTO_TYPE(type, func)` — removed; use `KARTO_TYPE(type)` instead
- `Referenced::Reference()`, `Referenced::Unreference()` — removed
- `CheckTypeRegistered()` — removed
- All `forEach` / `karto_forEach` macros — use range-based for loops

## Threading

TBB has been completely eliminated. The library uses:
- `std::mutex` / `std::lock_guard` for synchronization
- `std::thread` for parallel scan matching (partitioned by Y rows)
- `std::queue` + `std::condition_variable` for the grid set bank

Multi-threading is controlled by the `multiThreaded` constructor parameter on `OpenMapper`.

## Known Limitations

- **No solver**: Without a `ScanSolver` implementation, loop closure detection finds candidates but `CorrectPoses()` is a no-op. For operational areas under ~20m, drift is acceptable.
- **Global SensorRegistry**: Still exists as fallback. Use `SetLaserRangeFinder()` on scans for multi-mapper setups.
