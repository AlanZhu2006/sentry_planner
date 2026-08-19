cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(icp_probe)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)

add_executable(direct_icp_probe src/direct_icp_probe.cpp)
target_compile_features(direct_icp_probe PUBLIC cxx_std_17)
ament_target_dependencies(direct_icp_probe
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)
target_include_directories(direct_icp_probe SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(direct_icp_probe ${PCL_LIBRARIES})

add_executable(pcd_to_pgm src/pcd_to_pgm.cpp)
target_compile_features(pcd_to_pgm PUBLIC cxx_std_17)
target_include_directories(pcd_to_pgm SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_to_pgm ${PCL_LIBRARIES})

install(TARGETS direct_icp_probe pcd_to_pgm DESTINATION lib/${PROJECT_NAME})

ament_package()
