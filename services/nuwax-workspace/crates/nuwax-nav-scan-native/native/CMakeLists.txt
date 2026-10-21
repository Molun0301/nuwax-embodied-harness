cmake_minimum_required(VERSION 3.16)
project(nuwax_scan_native LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_library(nuwax_scan_native STATIC
  author_a/src/native_session.cpp author_a/src/planner_manager.cpp
  author_a/src/scan_replan_fsm.cpp author_a/src/grid_map.cpp author_a/src/raycast.cpp
  upstream/path_searching/src/dyn_a_star.cpp
  upstream/bspline_opt/src/bspline_optimizer.cpp
  upstream/bspline_opt/src/uniform_bspline.cpp
  upstream/traj_utils/src/polynomial_traj.cpp)
target_include_directories(nuwax_scan_native PUBLIC author_a/include include
  upstream/path_searching/include upstream/bspline_opt/include
  upstream/traj_utils/include upstream/plan_manage/include)
target_link_libraries(nuwax_scan_native PUBLIC Eigen3::Eigen)
# 本地测试源码按项目约定不入库；独立库默认可由已发布源码构建。
option(BUILD_TESTING "构建本地原生测试（需要本机测试源码）" OFF)
include(CTest)
if(BUILD_TESTING)
  add_executable(scan_native_geometry tests/geometry.cpp)
  target_link_libraries(scan_native_geometry PRIVATE nuwax_scan_native)
  add_test(NAME scan_native_geometry COMMAND scan_native_geometry)
  add_executable(scan_native_thin_door tests/door_geometry.cpp)
  target_link_libraries(scan_native_thin_door PRIVATE nuwax_scan_native)
  add_test(NAME scan_native_thin_door COMMAND scan_native_thin_door)
endif()
