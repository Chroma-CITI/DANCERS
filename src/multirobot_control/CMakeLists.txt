cmake_minimum_required(VERSION 3.8)
project(multirobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(obstacle_detector REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)

set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

add_executable(gz_odom_publisher src/gz_odom_publisher.cpp)

target_link_libraries(gz_odom_publisher
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

ament_target_dependencies(gz_odom_publisher
  px4_msgs
  rclcpp
  nav_msgs
)

add_executable(VAT_neighbors src/VAT_neighbors.cpp)

target_link_libraries(VAT_neighbors
  yaml-cpp
  Eigen3::Eigen
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

ament_target_dependencies(VAT_neighbors
  px4_msgs
  rclcpp
  sensor_msgs
  obstacle_detector
  nav_msgs
)


INSTALL(
  TARGETS 
  gz_odom_publisher
  VAT_neighbors
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
INSTALL(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)


ament_package()
