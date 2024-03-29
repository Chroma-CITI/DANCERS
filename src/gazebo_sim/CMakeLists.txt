cmake_minimum_required(VERSION 3.8)
project(gazebo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wshadow -Wunused -Wconversion -Wsign-conversion -Wduplicated-cond -Wduplicated-branches -Wnull-dereference -Wuseless-cast -Wdouble-promotion)
endif()

# Export a compile_commands.json file to feed VSCode's IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_net_sim_interfaces REQUIRED)
find_package(Boost REQUIRED system iostreams program_options filesystem)
find_package(protobuf_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    find_package(gz-sim7 REQUIRED)
    set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
    find_package(gz-common5 REQUIRED)
    set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
    
    add_executable(gazebo_sim src/gazebo_sim.cpp)
    add_executable(gazebo_only src/gazebo_only.cpp)
    target_link_libraries(gazebo_sim
            gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
            gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
            yaml-cpp
            )
    target_link_libraries(gazebo_only
            gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
            gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
            yaml-cpp
            )
endif()

ament_target_dependencies(gazebo_sim
  rclcpp
  ros_net_sim_interfaces
  Boost
  protobuf_msgs
  geometry_msgs
)

ament_target_dependencies(gazebo_only
  rclcpp
  ros_net_sim_interfaces
  Boost
  protobuf_msgs
  geometry_msgs
)


INSTALL(
  TARGETS 
  gazebo_sim
  gazebo_only
  DESTINATION lib/${PROJECT_NAME})

ament_package()
