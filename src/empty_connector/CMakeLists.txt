cmake_minimum_required(VERSION 3.8)
project(empty_connector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(connector_core REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)

add_executable(${PROJECT_NAME}_net src/empty_connector_net.cpp)

ament_target_dependencies(${PROJECT_NAME}_net
  rclcpp
  connector_core
)

target_include_directories(${PROJECT_NAME}_net PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/../common/include
)

add_executable(${PROJECT_NAME}_phy src/empty_connector_phy.cpp)

ament_target_dependencies(${PROJECT_NAME}_phy
  rclcpp
  connector_core
)

target_include_directories(${PROJECT_NAME}_phy PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/../common/include
)

install(TARGETS 
  ${PROJECT_NAME}_phy
  ${PROJECT_NAME}_net
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
