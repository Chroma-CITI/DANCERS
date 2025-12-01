cmake_minimum_required(VERSION 3.8)
project(flocking_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dancers_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(flocking_controller_vat 
  src/vat_controller_node.cpp  
  src/vat_controller.cpp
)

ament_target_dependencies(flocking_controller_vat 
  rclcpp 
  dancers_msgs
)

target_link_libraries(flocking_controller_vat
  Eigen3::Eigen
  yaml-cpp
)

target_include_directories(flocking_controller_vat PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/../common/include
)

INSTALL(TARGETS
  flocking_controller_vat
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
