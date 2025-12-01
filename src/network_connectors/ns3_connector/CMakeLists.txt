cmake_minimum_required(VERSION 3.8)
project(ns3_connector)

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
find_package(ns3 REQUIRED)
find_package(dancers_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ***** basic_wifi_adhoc *****
add_executable(basic_wifi_adhoc src/basic_wifi_adhoc.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/applications/waypoint_broadcast.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/applications/waypoint_broadcast.h
)

ament_target_dependencies(basic_wifi_adhoc
  rclcpp
  connector_core
  dancers_msgs
)

target_link_libraries(basic_wifi_adhoc
  Boost::iostreams
  ns3::libcore
  ns3::libwifi
  ns3::libmobility
  ns3::libinternet
  ns3::libpropagation
  ns3::libnetwork
  ns3::libapplications
  ns3::libbuildings
)

target_include_directories(basic_wifi_adhoc PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/../common/include
)

# ***** basic_wifi_adhoc_with_controller *****
add_executable(basic_wifi_adhoc_with_controller src/basic_wifi_adhoc_with_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/applications/flocking-application.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/applications/flocking-application.h
)

ament_target_dependencies(basic_wifi_adhoc_with_controller
  rclcpp
  connector_core
  dancers_msgs
  visualization_msgs
)

target_link_libraries(basic_wifi_adhoc_with_controller
  Boost::iostreams
  ns3::libcore
  ns3::libwifi
  ns3::libmobility
  ns3::libinternet
  ns3::libpropagation
  ns3::libnetwork
  ns3::libapplications
  ns3::libbuildings
)

target_include_directories(basic_wifi_adhoc_with_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/../common/include
)


INSTALL(
  TARGETS
  basic_wifi_adhoc
  basic_wifi_adhoc_with_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
