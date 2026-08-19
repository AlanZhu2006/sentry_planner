cmake_minimum_required(VERSION 3.8)
project(mid360_nav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf maps rviz web
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_navigation.sh
  scripts/fastlio_odom_adapter.py
  scripts/save_map.sh
  scripts/sentry_nav2_bridge.py
  scripts/start_mapping.sh
  scripts/start_navigation.sh
  scripts/start_wasd.sh
  scripts/wasd_teleop.py
  scripts/wasd_web_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
