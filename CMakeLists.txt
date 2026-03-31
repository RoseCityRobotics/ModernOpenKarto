cmake_minimum_required(VERSION 3.16)
project(OpenKarto VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(KARTO_SOURCE_FILES
  src/Any.cpp
  src/Event.cpp
  src/Exception.cpp
  src/Geometry.cpp
  src/Grid.cpp
  src/GridIndexLookup.cpp
  src/Identifier.cpp
  src/Logger.cpp
  src/Module.cpp
  src/Mutex.cpp
  src/Object.cpp
  src/Objects.cpp
  src/OccupancyGrid.cpp
  src/OpenMapper.cpp
  src/Parameter.cpp
  src/PoseTransform.cpp
  src/RangeTransform.cpp
  src/Referenced.cpp
  src/RigidBodyTransform.cpp
  src/Sensor.cpp
  src/SensorData.cpp
  src/SensorRegistry.cpp
  src/StringHelper.cpp
)

add_library(${PROJECT_NAME} ${KARTO_SOURCE_FILES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# TBB intentionally removed — target platform is ARM (Jetson Orin).
# Single-threaded fallbacks are used; Phase 8 will add std::thread parallelism.

# Install
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
  EXPORT OpenKartoTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h"
)

install(EXPORT OpenKartoTargets
  FILE OpenKartoTargets.cmake
  NAMESPACE OpenKarto::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenKarto
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/OpenKartoConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/OpenKartoConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenKarto
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/OpenKartoConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/OpenKartoConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/OpenKartoConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenKarto
)

# Tests
option(OPENKARTO_BUILD_TESTS "Build unit tests" ON)
if(OPENKARTO_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.16.0
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  add_subdirectory(test)
endif()
