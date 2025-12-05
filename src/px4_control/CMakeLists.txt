cmake_minimum_required(VERSION 3.8)
project(px4_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dancers_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(waypoint_control src/waypoint_control.cpp)

ament_target_dependencies(waypoint_control
  rclcpp
  dancers_msgs
  px4_msgs
  geometry_msgs
)

target_link_libraries(waypoint_control
  Eigen3::Eigen
)

target_include_directories(waypoint_control PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/../common/include
)

INSTALL(
  TARGETS
  waypoint_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
