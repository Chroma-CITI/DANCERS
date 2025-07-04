cmake_minimum_required(VERSION 3.8)
project(mini_dancers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dancers_msgs REQUIRED)
find_package(protobuf_msgs REQUIRED)
find_package(Boost REQUIRED system iostreams filesystem)
find_package(std_msgs REQUIRED)

add_executable(mini_dancers src/mini_dancers.cpp)

ament_target_dependencies(mini_dancers
  rclcpp
  visualization_msgs
  geometry_msgs
  dancers_msgs
  protobuf_msgs
  Boost
  std_msgs
)

target_link_libraries(mini_dancers 
  Eigen3::Eigen
  yaml-cpp
)

include_directories(include/${PROJECT_NAME})
include_directories(include/${PROJECT_NAME}/util)
include_directories(include/${PROJECT_NAME}/command_algorithms)
# hard-coded include path makes this file fail if moved
include_directories(../../../_include_all)

INSTALL(
  TARGETS mini_dancers DESTINATION lib/${PROJECT_NAME}
)

INSTALL(
  DIRECTORY include/ DESTINATION include
)

ament_export_include_directories(include)

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

ament_package()
