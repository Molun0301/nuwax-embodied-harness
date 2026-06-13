cmake_minimum_required(VERSION 3.5)
project(go2_nav)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/cmd_vel_bridge.py
  scripts/odom_tf_broadcaster.py
  scripts/cloud_to_scan.py
  scripts/frontier_explorer.py
  scripts/plan_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
