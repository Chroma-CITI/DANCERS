cmake_minimum_required(VERSION 3.8)
project(gazebo_connector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(connector_core REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(dancers_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(geometry_msgs REQUIRED)

set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

add_executable(gazebo_connector src/gazebo_connector.cpp)

ament_target_dependencies(gazebo_connector
  rclcpp
  connector_core
  dancers_msgs
  geometry_msgs
)

target_link_libraries(gazebo_connector
  Boost::iostreams
  Eigen3::Eigen
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

target_include_directories(gazebo_connector PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/../common/include
)

target_compile_features(gazebo_connector PRIVATE cxx_std_17)

INSTALL(
  TARGETS
  gazebo_connector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
