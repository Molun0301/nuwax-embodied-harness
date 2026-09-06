cmake_minimum_required(VERSION 3.14)
project(pointlio_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
add_definitions(-DROOT_DIR="/tmp/pointlio_")

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/result" CACHE PATH "" FORCE)
endif()

find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# MP defines (same logic as FAST-LIO)
message("CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  if(N GREATER 4)
    add_definitions(-DMP_EN -DMP_PROC_NUM=3)
  elseif(N GREATER 3)
    add_definitions(-DMP_EN -DMP_PROC_NUM=2)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

include(FetchContent)

# FAST-LIO-NON-ROS (pass -DFASTLIO_DIR=<path> or use local copy)
if(NOT FASTLIO_DIR)
  set(FASTLIO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/fast-lio-non-ros)
endif()

# flake 输入是锁定的上游源码；不同版本的检出根目录可能变化，不能假定单一布局。
set(FASTLIO_SOURCE_DIR "")
foreach(CANDIDATE
  "${FASTLIO_DIR}"
  "${FASTLIO_DIR}/pointlio"
  "${FASTLIO_DIR}/src"
)
  if(EXISTS "${CANDIDATE}/src/preprocess.cpp")
    set(FASTLIO_SOURCE_DIR "${CANDIDATE}")
    break()
  endif()
endforeach()
if(NOT FASTLIO_SOURCE_DIR)
  message(FATAL_ERROR
    "FASTLIO_DIR does not contain src/preprocess.cpp: ${FASTLIO_DIR}. "
    "Pass -DFASTLIO_DIR to the locked Point-LIO source checkout.")
endif()

foreach(REQUIRED_SOURCE preprocess.cpp Estimator.cpp parameters.cpp)
  if(NOT EXISTS "${FASTLIO_SOURCE_DIR}/src/${REQUIRED_SOURCE}")
    message(FATAL_ERROR
      "Point-LIO source is missing src/${REQUIRED_SOURCE} under ${FASTLIO_SOURCE_DIR}")
  endif()
endforeach()

# 在构建目录统一应用Point修复，普通构建和Nix共用同一入口。
find_program(NUWAX_PATCH_EXECUTABLE patch)
if(NOT NUWAX_PATCH_EXECUTABLE)
  message(FATAL_ERROR "patch is required to apply the Point-LIO fixes")
endif()
set(POINTLIO_PATCHED_DIR "${CMAKE_CURRENT_BINARY_DIR}/pointlio-initialized")
set(POINTLIO_PATCHES
  "${CMAKE_CURRENT_SOURCE_DIR}/fastlio-resize-darwin.patch"
  "${CMAKE_CURRENT_SOURCE_DIR}/pointlio-imu-initialization.patch"
  "${CMAKE_CURRENT_SOURCE_DIR}/pointlio-empty-neighbours.patch")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${POINTLIO_PATCHES})
foreach(PATCHED_SOURCE pointlio.hpp laserMapping.hpp Estimator.cpp)
  configure_file("${FASTLIO_SOURCE_DIR}/src/${PATCHED_SOURCE}"
    "${POINTLIO_PATCHED_DIR}/src/${PATCHED_SOURCE}" COPYONLY)
endforeach()
foreach(POINTLIO_PATCH IN LISTS POINTLIO_PATCHES)
  execute_process(
    COMMAND "${NUWAX_PATCH_EXECUTABLE}" --batch --forward --fuzz=0 --dry-run -p1
      -i "${POINTLIO_PATCH}"
    WORKING_DIRECTORY "${POINTLIO_PATCHED_DIR}"
    RESULT_VARIABLE POINTLIO_PATCH_CHECK_RESULT
    OUTPUT_VARIABLE POINTLIO_PATCH_CHECK_OUTPUT
    ERROR_VARIABLE POINTLIO_PATCH_CHECK_ERROR
  )
  if(POINTLIO_PATCH_CHECK_RESULT EQUAL 0)
    execute_process(
      COMMAND "${NUWAX_PATCH_EXECUTABLE}" --batch --forward --fuzz=0 -p1
        -i "${POINTLIO_PATCH}"
      WORKING_DIRECTORY "${POINTLIO_PATCHED_DIR}"
      RESULT_VARIABLE POINTLIO_PATCH_RESULT
      OUTPUT_VARIABLE POINTLIO_PATCH_OUTPUT
      ERROR_VARIABLE POINTLIO_PATCH_ERROR
    )
    if(NOT POINTLIO_PATCH_RESULT EQUAL 0)
      message(FATAL_ERROR
        "Point-LIO patch failed (${POINTLIO_PATCH}): ${POINTLIO_PATCH_OUTPUT}${POINTLIO_PATCH_ERROR}")
    endif()
  else()
    # 旧构建缓存可能已含这份补丁；只接受能精确反向核对的已应用状态。
    execute_process(
      COMMAND "${NUWAX_PATCH_EXECUTABLE}" --batch --reverse --fuzz=0 --dry-run -p1
        -i "${POINTLIO_PATCH}"
      WORKING_DIRECTORY "${POINTLIO_PATCHED_DIR}"
      RESULT_VARIABLE POINTLIO_REVERSE_CHECK_RESULT
      OUTPUT_VARIABLE POINTLIO_REVERSE_CHECK_OUTPUT
      ERROR_VARIABLE POINTLIO_REVERSE_CHECK_ERROR
    )
    if(NOT POINTLIO_REVERSE_CHECK_RESULT EQUAL 0)
      message(FATAL_ERROR
        "Point-LIO patch does not match source (${POINTLIO_PATCH}): "
        "${POINTLIO_PATCH_CHECK_OUTPUT}${POINTLIO_PATCH_CHECK_ERROR}"
        "${POINTLIO_REVERSE_CHECK_OUTPUT}${POINTLIO_REVERSE_CHECK_ERROR}")
    endif()
    message(STATUS "Point-LIO source already contains patch: ${POINTLIO_PATCH}")
  endif()
endforeach()

FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

find_package(Eigen3 REQUIRED)

# PCL (only components we need — avoid full PCL which drags in VTK via io)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters)

# glog 由 Nix 提供；只接受其 CMake 包或 pkg-config 导入目标，禁止从裸头文件和
# 任意同名库拼装目标，以免配置阶段接受不匹配的头库组合。
find_package(glog CONFIG QUIET)
if(TARGET glog::glog)
  set(NUWAX_GLOG_TARGET glog::glog)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GLOG QUIET IMPORTED_TARGET libglog)
  if(NOT TARGET PkgConfig::GLOG)
    pkg_check_modules(GLOG QUIET IMPORTED_TARGET glog)
  endif()
  if(TARGET PkgConfig::GLOG)
    set(NUWAX_GLOG_TARGET PkgConfig::GLOG)
  else()
    message(FATAL_ERROR
      "glog CMake package and pkg-config provider unavailable; "
      "install the locked Nix glog dependency")
  endif()
endif()

find_library(LIVOX_SDK livox_lidar_sdk_shared)
if(NOT LIVOX_SDK)
  message(FATAL_ERROR "Livox SDK2 not found. Available via nix flake in lidar/livox/")
endif()
get_filename_component(LIVOX_SDK_LIB_DIR ${LIVOX_SDK} DIRECTORY)
get_filename_component(LIVOX_SDK_PREFIX ${LIVOX_SDK_LIB_DIR} DIRECTORY)
set(LIVOX_SDK_INCLUDE_DIR ${LIVOX_SDK_PREFIX}/include)

add_executable(pointlio_native
  main.cpp
  ${FASTLIO_SOURCE_DIR}/src/preprocess.cpp
  ${POINTLIO_PATCHED_DIR}/src/Estimator.cpp
  ${FASTLIO_SOURCE_DIR}/src/parameters.cpp
)

if(NOT LIVOX_COMMON_DIR)
  set(LIVOX_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
endif()

target_include_directories(pointlio_native PRIVATE
  ${POINTLIO_PATCHED_DIR}/src
  ${FASTLIO_SOURCE_DIR}/include
  ${FASTLIO_SOURCE_DIR}/include/IKFoM/IKFoM_toolkit
  ${FASTLIO_SOURCE_DIR}/src
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${LIVOX_COMMON_DIR}
  ${LIVOX_SDK_INCLUDE_DIR}
)

target_link_libraries(pointlio_native PRIVATE
  ${LCM_LIBRARIES}
  ${LIVOX_SDK}
  ${PCL_LIBRARIES}
  ${NUWAX_GLOG_TARGET}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(pointlio_native PRIVATE OpenMP::OpenMP_CXX)
endif()

target_link_directories(pointlio_native PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS pointlio_native DESTINATION bin)
