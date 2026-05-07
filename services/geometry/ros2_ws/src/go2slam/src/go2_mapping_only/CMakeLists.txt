cmake_minimum_required(VERSION 3.8)
project(go2_mapping_only)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS registration)

add_executable(go2_map_builder src/go2_map_builder.cpp)
target_include_directories(go2_map_builder PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(go2_map_builder ${PCL_LIBRARIES})
ament_target_dependencies(go2_map_builder
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
)

install(TARGETS
  go2_map_builder
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
