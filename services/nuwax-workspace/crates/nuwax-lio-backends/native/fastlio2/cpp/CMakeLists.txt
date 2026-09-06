cmake_minimum_required(VERSION 3.14)
project(fastlio2_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/result" CACHE PATH "" FORCE)
endif()

# OpenMP for parallel processing
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

# Fetch dependencies
include(FetchContent)

# FAST-LIO-NON-ROS (pass -DFASTLIO_DIR=<path> or use local copy)
if(NOT FASTLIO_DIR)
  set(FASTLIO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/fast-lio-non-ros)
endif()

# flake 输入是锁定的上游源码；不同版本的检出根目录可能变化，不能假定单一布局。
set(FASTLIO_SOURCE_DIR "")
foreach(CANDIDATE
  "${FASTLIO_DIR}"
  "${FASTLIO_DIR}/fastlio2"
  "${FASTLIO_DIR}/fast_lio"
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
    "Pass -DFASTLIO_DIR to the locked FAST-LIO source checkout.")
endif()

set(FASTLIO_IKD_TREE_SOURCE "")
foreach(CANDIDATE
  "${FASTLIO_SOURCE_DIR}/include/ikd-Tree/ikd_Tree.cpp"
  "${FASTLIO_SOURCE_DIR}/src/ikd-Tree/ikd_Tree.cpp"
  "${FASTLIO_SOURCE_DIR}/src/ikd_Tree.cpp"
)
  if(EXISTS "${CANDIDATE}")
    set(FASTLIO_IKD_TREE_SOURCE "${CANDIDATE}")
    break()
  endif()
endforeach()
if(NOT FASTLIO_IKD_TREE_SOURCE)
  message(FATAL_ERROR
    "FAST-LIO source is missing ikd_Tree.cpp under ${FASTLIO_SOURCE_DIR}")
endif()

# 修复锁定上游的首帧状态初始化，只在构建目录内生成实际使用的头文件。
# CMake 与 Nix 共用这一入口，不修改外部源码检出，也不依赖其临时手工补丁。
find_program(NUWAX_PATCH_EXECUTABLE patch)
if(NOT NUWAX_PATCH_EXECUTABLE)
  message(FATAL_ERROR "patch is required to apply the FAST-LIO initialization fix")
endif()
set(FASTLIO_PATCHED_DIR "${CMAKE_CURRENT_BINARY_DIR}/fastlio-initialized")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/fastlio-initialization.patch")
foreach(HEADER fast_lio.hpp laserMapping.hpp IMU_Processing.hpp)
  configure_file("${FASTLIO_SOURCE_DIR}/src/${HEADER}"
    "${FASTLIO_PATCHED_DIR}/src/${HEADER}" COPYONLY)
endforeach()
execute_process(
  COMMAND "${NUWAX_PATCH_EXECUTABLE}" --batch --forward -p1
    -i "${CMAKE_CURRENT_SOURCE_DIR}/fastlio-initialization.patch"
  WORKING_DIRECTORY "${FASTLIO_PATCHED_DIR}"
  RESULT_VARIABLE FASTLIO_PATCH_RESULT
  OUTPUT_VARIABLE FASTLIO_PATCH_OUTPUT
  ERROR_VARIABLE FASTLIO_PATCH_ERROR
)
if(NOT FASTLIO_PATCH_RESULT EQUAL 0)
  message(FATAL_ERROR
    "FAST-LIO initialization patch failed: ${FASTLIO_PATCH_OUTPUT}${FASTLIO_PATCH_ERROR}")
endif()

# dimos-lcm C++ message headers
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

# LCM
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# Eigen3
find_package(Eigen3 REQUIRED)

# PCL (only components we need — avoid full PCL which drags in VTK via io)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters)

# Livox SDK2 (from nix or /usr/local fallback)
find_library(LIVOX_SDK livox_lidar_sdk_shared)
if(NOT LIVOX_SDK)
  message(FATAL_ERROR "Livox SDK2 not found. Available via nix flake in lidar/livox/")
endif()
get_filename_component(LIVOX_SDK_LIB_DIR ${LIVOX_SDK} DIRECTORY)
get_filename_component(LIVOX_SDK_PREFIX ${LIVOX_SDK_LIB_DIR} DIRECTORY)
set(LIVOX_SDK_INCLUDE_DIR ${LIVOX_SDK_PREFIX}/include)

add_executable(fastlio2_native
  main.cpp
  ${FASTLIO_SOURCE_DIR}/src/preprocess.cpp
  ${FASTLIO_IKD_TREE_SOURCE}
)

# Shared Livox common headers (livox_sdk_config.hpp etc.)
if(NOT LIVOX_COMMON_DIR)
  set(LIVOX_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
endif()

target_include_directories(fastlio2_native PRIVATE
  ${FASTLIO_PATCHED_DIR}/src
  ${FASTLIO_SOURCE_DIR}/include
  ${FASTLIO_SOURCE_DIR}/src
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${LIVOX_COMMON_DIR}
  ${LIVOX_SDK_INCLUDE_DIR}
)

target_link_libraries(fastlio2_native PRIVATE
  ${LCM_LIBRARIES}
  ${LIVOX_SDK}
  ${PCL_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlio2_native PRIVATE OpenMP::OpenMP_CXX)
endif()

target_link_directories(fastlio2_native PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS fastlio2_native DESTINATION bin)
