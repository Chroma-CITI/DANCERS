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
find_package(Boost REQUIRED system iostreams program_options filesystem)
find_package(protobuf_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dancers_msgs REQUIRED)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    find_package(gz-sim8 REQUIRED)
    set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
    find_package(gz-common5 REQUIRED)
    set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
    
    add_executable(gazebo_sim src/gazebo_sim.cpp)
    target_link_libraries(gazebo_sim
            gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
            gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
            yaml-cpp
            )
endif()

ament_target_dependencies(gazebo_sim
  rclcpp
  Boost
  protobuf_msgs
  geometry_msgs
  nav_msgs
  dancers_msgs
)

include_directories(include/${PROJECT_NAME})

# hard-coded include path makes this file fail if moved
include_directories(../../../_include_all)

INSTALL(
  TARGETS 
  gazebo_sim
  DESTINATION lib/${PROJECT_NAME})

ament_package()
