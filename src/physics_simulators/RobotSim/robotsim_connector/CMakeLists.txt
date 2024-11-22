cmake_minimum_required(VERSION 3.8)
project(robotsim_connector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED system iostreams program_options filesystem)
find_package(protobuf_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(robotsim_connector src/robotsim_connector.cpp)

ament_target_dependencies(robotsim_connector
  rclcpp
  Boost
  protobuf_msgs)

include_directories(include/${PROJECT_NAME})

target_link_libraries(robotsim_connector
  yaml-cpp)

# hard-coded include path makes this file fail if moved
include_directories(../../../_include_all)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

INSTALL(
  TARGETS 
  robotsim_connector
  DESTINATION lib/${PROJECT_NAME})


ament_package()
