cmake_minimum_required(VERSION 3.8)
project(go2_navigation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PCL REQUIRED COMPONENTS io filters registration)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(navigation src/navigation.cpp)
target_include_directories(navigation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(navigation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  navigation
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "nav_msgs"
)

add_executable(front_align_cmd_vel_gate src/front_align_cmd_vel_gate.cpp)
target_include_directories(front_align_cmd_vel_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(front_align_cmd_vel_gate PUBLIC cxx_std_17)
ament_target_dependencies(
  front_align_cmd_vel_gate
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)

add_executable(pcd_initial_pose_publisher src/pcd_initial_pose_publisher.cpp)
target_include_directories(pcd_initial_pose_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_compile_features(pcd_initial_pose_publisher PUBLIC cxx_std_17)
ament_target_dependencies(
  pcd_initial_pose_publisher
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
target_link_libraries(pcd_initial_pose_publisher ${PCL_LIBRARIES})

add_executable(initial_pose_spin_relocalizer src/initial_pose_spin_relocalizer.cpp)
target_include_directories(initial_pose_spin_relocalizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(initial_pose_spin_relocalizer PUBLIC cxx_std_17)
ament_target_dependencies(
  initial_pose_spin_relocalizer
  "rclcpp"
  "geometry_msgs"
)

# ClimbStairsTask plugin
add_library(climb_stairs_task SHARED
  src/climb_stairs_task.cpp
)
target_include_directories(climb_stairs_task PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(climb_stairs_task PUBLIC cxx_std_17)
ament_target_dependencies(
  climb_stairs_task
  "rclcpp"
  "rclcpp_lifecycle"
  "geometry_msgs"
  "nav2_core"
  "nav2_util"
  "pluginlib"
)

pluginlib_export_plugin_description_file(nav2_core climb_stairs_plugin.xml)

install(TARGETS navigation front_align_cmd_vel_gate pcd_initial_pose_publisher initial_pose_spin_relocalizer climb_stairs_task
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_pcd_initial_pose_utils
    test/test_pcd_initial_pose_utils.cpp
  )
  target_include_directories(test_pcd_initial_pose_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(test_pcd_initial_pose_utils
    "rclcpp"
    "geometry_msgs"
    "tf2"
    "tf2_geometry_msgs"
  )
endif()

ament_package()
