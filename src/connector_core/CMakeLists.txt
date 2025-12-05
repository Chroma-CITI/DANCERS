cmake_minimum_required(VERSION 3.8)
project(connector_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(protobuf_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)

# Create INTERFACE target (header-only)
add_library(${PROJECT_NAME} INTERFACE)

# Expose include directories to downstream packages
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../common/include>
  $<INSTALL_INTERFACE:include>
)

# Propagate dependencies (so downstream doesn't need to re-find them manually)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
  rclcpp
  protobuf_msgs
  )
  
# Link Boost libraries to the INTERFACE target so they propagate
target_link_libraries(${PROJECT_NAME} INTERFACE
  yaml-cpp
  Boost::iostreams
)

# Export target and install headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../common/include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp yaml-cpp protobuf_msgs Boost)
ament_export_include_directories(include)

ament_package()
