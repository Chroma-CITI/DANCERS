cmake_minimum_required(VERSION 3.8)
project(flocking_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(dancers_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(occupancy_grid 
            src/occupancy_grid.cpp
            src/grid_path_planner.cpp)

add_executable(flocking_controller_vat 
               src/vat_controller_node.cpp 
               src/vat_controller.cpp)


ament_target_dependencies(flocking_controller_vat 
  rclcpp 
  dancers_msgs 
  geometry_msgs
  nav_msgs
)

ament_target_dependencies(occupancy_grid 
  nav_msgs
  geometry_msgs
)

target_link_libraries(occupancy_grid 
  Eigen3::Eigen
)

target_link_libraries(flocking_controller_vat 
  Eigen3::Eigen
  yaml-cpp
  occupancy_grid
)


install(TARGETS
  flocking_controller_vat
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(include)

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
