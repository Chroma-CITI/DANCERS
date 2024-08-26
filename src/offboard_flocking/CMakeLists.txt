cmake_minimum_required(VERSION 3.8)
project(offboard_flocking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(obstacle_detector REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(gz-common5 REQUIRED)


set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

add_executable(com_test_pub src/com_test_dds_publisher.cpp)
add_executable(com_test_sub src/com_test_dds_subscriber.cpp)

ament_target_dependencies(com_test_pub
  # px4_msgs
  rclcpp
  # turtlesim
  std_msgs
)

ament_target_dependencies(com_test_sub
  # px4_msgs
  rclcpp
  # turtlesim
  std_msgs
)

add_executable(VAT_flocking src/VAT_flocking.cpp)

target_link_libraries(VAT_flocking
  yaml-cpp
  Eigen3::Eigen
)

ament_target_dependencies(VAT_flocking
  px4_msgs
  rclcpp
  sensor_msgs
  obstacle_detector
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
  )

add_executable(simple_control src/simple_control_pos.cpp)

target_link_libraries(simple_control
  yaml-cpp
  Eigen3::Eigen
)

ament_target_dependencies(simple_control
  px4_msgs
  rclcpp
  visualization_msgs
)

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

add_executable(gz_pos_saver src/gz_pos_saver.cpp)

target_link_libraries(gz_pos_saver
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

ament_target_dependencies(gz_pos_saver
  rclcpp
)

add_executable(com_test_broadcast src/com_test_udp_broadcast.cpp)

ament_target_dependencies(com_test_broadcast
  rclcpp
  nav_msgs)

add_executable(VAT_UDP_flocking src/VAT_UDP_flocking.cpp)

target_link_libraries(VAT_UDP_flocking
  yaml-cpp
  Eigen3::Eigen
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

ament_target_dependencies(VAT_UDP_flocking
  px4_msgs
  rclcpp
  sensor_msgs
  obstacle_detector
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
)


add_executable(px4_pos_saver src/px4_pos_saver.cpp)
target_link_libraries(px4_pos_saver
  Eigen3::Eigen
)
ament_target_dependencies(px4_pos_saver
  px4_msgs
  rclcpp
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
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
)


INSTALL(
  TARGETS 
  VAT_flocking
  simple_control
  gz_odom_publisher
  gz_pos_saver
  com_test_pub
  com_test_sub
  com_test_broadcast
  VAT_UDP_flocking
  px4_pos_saver
  VAT_neighbors
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
INSTALL(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)


ament_package()
