cmake_minimum_required(VERSION 3.8)
project(coordinator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wshadow -Wunused -Wconversion -Wsign-conversion -Wduplicated-cond -Wduplicated-branches -Wnull-dereference -Wuseless-cast -Wdouble-promotion)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED system iostreams program_options filesystem fiber)
find_package(yaml-cpp REQUIRED)
find_package(protobuf_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_executable(coordinator src/coordinator.cpp)

ament_target_dependencies(coordinator
  rclcpp
  Boost
  protobuf_msgs
  rosgraph_msgs
)
target_link_libraries(coordinator
  yaml-cpp)

include_directories(include/${PROJECT_NAME})

INSTALL(
  TARGETS
  coordinator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
