cmake_minimum_required(VERSION 3.8)
project(ns-3_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wshadow -Wunused -Wconversion -Wsign-conversion -Wduplicated-cond -Wduplicated-branches -Wnull-dereference -Wuseless-cast -Wdouble-promotion)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(protobuf_msgs REQUIRED)
find_package(Boost REQUIRED iostreams filesystem system)
find_package(ns3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# add_executable(ns-3_sim_wifi_adhoc src/ns-3_sim_wifi_adhoc.cpp 
#   include/${PROJECT_NAME}/wifi-application.cpp 
#   include/${PROJECT_NAME}/wifi-application.h
#   include/${PROJECT_NAME}/chain-flocking-application.cpp
#   include/${PROJECT_NAME}/chain-flocking-application.h
#   include/${PROJECT_NAME}/pick-neighbors-header.cpp
#   include/${PROJECT_NAME}/pick-neighbors-header.h)
# target_link_libraries(ns-3_sim_wifi_adhoc
#   yaml-cpp
#   ns3::libcore
#   ns3::libwifi
#   ns3::libmobility
#   ns3::libolsr
#   ns3::libaodv
#   ns3::libinternet
#   ns3::libbuildings
#   ns3::libpropagation
#   ns3::libspectrum
#   ns3::libnetwork
#   ns3::libapplications
#   ns3::libcsma
#   ns3::libtap-bridge)
# ament_target_dependencies(ns-3_sim_wifi_adhoc
#   Boost 
#   protobuf_msgs
#   rclcpp)

add_executable(adhoc_chain_flocking src/adhoc_chain_flocking.cpp 
  include/${PROJECT_NAME}/wifi-application.cpp 
  include/${PROJECT_NAME}/wifi-application.h
  include/${PROJECT_NAME}/chain-flocking-application.cpp
  include/${PROJECT_NAME}/chain-flocking-application.h
  include/${PROJECT_NAME}/pick-neighbors-header.cpp
  include/${PROJECT_NAME}/pick-neighbors-header.h
  include/${PROJECT_NAME}/flocking-application.cpp
  include/${PROJECT_NAME}/flocking-application.h
  )
target_link_libraries(adhoc_chain_flocking
  yaml-cpp
  ns3::libcore
  ns3::libwifi
  ns3::libmobility
  ns3::libolsr
  ns3::libaodv
  ns3::libdsdv
  ns3::libdsr
  ns3::libinternet
  ns3::libbuildings
  ns3::libpropagation
  ns3::libspectrum
  ns3::libnetwork
  ns3::libapplications
  ns3::libcsma
  ns3::libtap-bridge)
ament_target_dependencies(adhoc_chain_flocking
  Boost 
  protobuf_msgs
  rclcpp)

# message("Boost version " ${Boost_VERSION_STRING})

include_directories(include/${PROJECT_NAME})
include_directories(../_include_all)

INSTALL(
  TARGETS 
  # ns-3_sim_adhoc
  # ns-3_sim_adhoc_no_tap
  # ns-3_sim_ap
  # ns-3_sim_csma
  # ns-3_only_adhoc
  # ns-3_only_adhoc_no_tap
  # ns-3_only_ap
  # ns-3_only_csma
  # ns-3_sim_wifi_adhoc
  adhoc_chain_flocking
  DESTINATION lib/${PROJECT_NAME})

ament_package()
