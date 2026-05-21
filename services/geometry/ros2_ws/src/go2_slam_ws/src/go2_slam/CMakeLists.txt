cmake_minimum_required(VERSION 3.5)
project(go2_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
