cmake_minimum_required(VERSION 3.16)
project(nuwax_slamtec LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(Threads REQUIRED)
# 保持上游源码集合；只编译当前Linux设备接口，不引入其它操作系统实现。
file(GLOB UNPACKERS CONFIGURE_DEPENDS sdk/src/dataunpacker/*.cpp sdk/src/dataunpacker/unpacker/*.cpp)
add_library(nuwax_slamtec SHARED bridge.cpp sdk/src/sl_lidar_driver.cpp sdk/src/hal/thread.cpp
  sdk/src/sl_crc.cpp sdk/src/sl_serial_channel.cpp sdk/src/sl_lidarprotocol_codec.cpp
  sdk/src/sl_async_transceiver.cpp sdk/src/sl_tcp_channel.cpp sdk/src/sl_udp_channel.cpp
  sdk/src/arch/linux/net_serial.cpp sdk/src/arch/linux/net_socket.cpp sdk/src/arch/linux/timer.cpp ${UNPACKERS})
target_include_directories(nuwax_slamtec PRIVATE sdk/include sdk/src sdk/src/dataunpacker sdk/src/dataunpacker/unpacker)
target_link_libraries(nuwax_slamtec PRIVATE Threads::Threads)
set_target_properties(nuwax_slamtec PROPERTIES VERSION 1.0.0 SOVERSION 1)
install(TARGETS nuwax_slamtec LIBRARY DESTINATION lib)
