cmake_minimum_required(VERSION 3.8)
project(mini_dancers)

# C++ 20 is required for ns-3.42
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Optional: Ensures standard compliance without compiler-specific extensions

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
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ***** mini_dancers *****
add_executable(mini_dancers src/mini_dancers.cpp)

ament_target_dependencies(mini_dancers
  rclcpp
  connector_core
  dancers_msgs
  visualization_msgs
  geometry_msgs
)

target_link_libraries(mini_dancers
  Boost::iostreams
  Eigen3::Eigen
)

target_include_directories(mini_dancers PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/../common/include
)

# ***** mini_dancers_velocity *****
add_executable(mini_dancers_velocity src/mini_dancers_velocity.cpp)

ament_target_dependencies(mini_dancers_velocity
  rclcpp
  connector_core
  dancers_msgs
  visualization_msgs
  geometry_msgs
)

target_link_libraries(mini_dancers_velocity
  Boost::iostreams
  Eigen3::Eigen
)

target_include_directories(mini_dancers_velocity PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/../common/include
)

INSTALL(
  TARGETS
  mini_dancers
  mini_dancers_velocity
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
