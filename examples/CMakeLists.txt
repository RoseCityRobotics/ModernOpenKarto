include(FetchContent)
FetchContent_Declare(rplidar_sdk
  GIT_REPOSITORY https://github.com/Slamtec/rplidar_sdk.git
  GIT_TAG master
)
FetchContent_MakeAvailable(rplidar_sdk)

# Build RPLidar SDK as a static library from source
# (the SDK ships with Makefiles only, no CMakeLists.txt)
set(RPLIDAR_SRC ${rplidar_sdk_SOURCE_DIR}/sdk/src)

add_library(rplidar_sdk_lib STATIC
  ${RPLIDAR_SRC}/sl_lidar_driver.cpp
  ${RPLIDAR_SRC}/hal/thread.cpp
  ${RPLIDAR_SRC}/sl_crc.cpp
  ${RPLIDAR_SRC}/sl_serial_channel.cpp
  ${RPLIDAR_SRC}/sl_lidarprotocol_codec.cpp
  ${RPLIDAR_SRC}/sl_async_transceiver.cpp
  ${RPLIDAR_SRC}/sl_tcp_channel.cpp
  ${RPLIDAR_SRC}/sl_udp_channel.cpp
  ${RPLIDAR_SRC}/dataunpacker/dataunpacker.cpp
  ${RPLIDAR_SRC}/dataunpacker/unpacker/handler_capsules.cpp
  ${RPLIDAR_SRC}/dataunpacker/unpacker/handler_hqnode.cpp
  ${RPLIDAR_SRC}/dataunpacker/unpacker/handler_normalnode.cpp
  ${RPLIDAR_SRC}/arch/linux/net_serial.cpp
  ${RPLIDAR_SRC}/arch/linux/net_socket.cpp
  ${RPLIDAR_SRC}/arch/linux/timer.cpp
)

target_include_directories(rplidar_sdk_lib SYSTEM PUBLIC
  ${rplidar_sdk_SOURCE_DIR}/sdk/include
  ${rplidar_sdk_SOURCE_DIR}/sdk/src
)

target_include_directories(rplidar_sdk_lib SYSTEM PRIVATE
  ${RPLIDAR_SRC}/dataunpacker
  ${RPLIDAR_SRC}/dataunpacker/unpacker
  ${RPLIDAR_SRC}/arch/linux
)

target_link_libraries(rplidar_sdk_lib PRIVATE pthread)

# Demo executable
add_executable(rplidar_slam_demo rplidar_slam_demo.cpp)
target_link_libraries(rplidar_slam_demo PRIVATE OpenKarto rplidar_sdk_lib)
