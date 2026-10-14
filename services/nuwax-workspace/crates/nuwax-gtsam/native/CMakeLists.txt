cmake_minimum_required(VERSION 3.16)
project(nuwax_gtsam_core LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(Threads REQUIRED)
find_package(GTSAM 4.2 QUIET)
if(NOT GTSAM_FOUND)
  include(FetchContent)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_TIMING_ALWAYS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "" FORCE)
  set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "" FORCE)
  set(GTSAM_WITH_TBB OFF CACHE BOOL "" FORCE)
  set(GTSAM_POSE3_EXPMAP ON CACHE BOOL "" FORCE)
  # 与Dimos锁定源码使用相同编译兼容选项，保留告警本身。
  add_compile_options(-Wno-error=array-bounds)
  FetchContent_Declare(gtsam_source
    URL https://codeload.github.com/borglab/gtsam/tar.gz/1a9792a7ede244850a413739557635b606f295c0
    URL_HASH SHA256=62f9624662e54b6f5aff03784504f790389bed85d2dee1648ba839bc78df6407
    PATCH_COMMAND ${CMAKE_COMMAND} "-DNUWAX_GTSAM_SOURCE_DIR=<SOURCE_DIR>"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/eigen_compat.cmake")
  FetchContent_MakeAvailable(gtsam_source)
endif()
# Ubuntu20/22的pkg-config名称带版本后缀，使用PCL官方CMake配置统一解析依赖。
find_package(PCL 1.8 REQUIRED COMPONENTS common kdtree registration)
find_package(OpenMP REQUIRED COMPONENTS CXX)
add_library(nuwax_gtsam STATIC solver.cpp registration.cpp)
target_include_directories(nuwax_gtsam PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})
target_compile_options(nuwax_gtsam PUBLIC ${PCL_DEFINITIONS})
# GTSAM静态核心使用线程；由共享库声明，兼容pthread尚未并入libc的目标系统。
target_link_libraries(nuwax_gtsam PUBLIC gtsam ${PCL_LIBRARIES} Threads::Threads OpenMP::OpenMP_CXX)
# Rust调用方读取CMake实际选定的库位置；C++调用方直接复用目标传递依赖。
file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/gtsam-library.txt CONTENT "$<TARGET_FILE:gtsam>\n$<TARGET_FILE:metis-gtsam>\n$<TARGET_FILE:cephes-gtsam>\n")
# 静态库不会携带动态依赖；把目标编译器选择的OpenMP运行库传给Rust最终链接。
set(NUWAX_OPENMP_LIBRARIES "")
foreach(NUWAX_OPENMP_NAME IN LISTS OpenMP_CXX_LIB_NAMES)
  string(APPEND NUWAX_OPENMP_LIBRARIES "${NUWAX_OPENMP_NAME}=${OpenMP_${NUWAX_OPENMP_NAME}_LIBRARY}\n")
endforeach()
file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/gtsam-openmp-libraries.txt CONTENT "${NUWAX_OPENMP_LIBRARIES}")
