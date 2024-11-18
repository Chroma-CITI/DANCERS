cmake_minimum_required(VERSION 3.8)
project(ns-3_sim)

# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)  # Optional: Ensures standard compliance without compiler-specific extensions

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

add_executable(fake_neighborhood src/fake_neighborhood.cpp
  include/${PROJECT_NAME}/wifi-application.cpp 
  include/${PROJECT_NAME}/wifi-application.h
  include/${PROJECT_NAME}/chain-flocking-application.cpp
  include/${PROJECT_NAME}/chain-flocking-application.h
  include/${PROJECT_NAME}/pick-neighbors-header.cpp
  include/${PROJECT_NAME}/pick-neighbors-header.h
  include/${PROJECT_NAME}/flocking-application.cpp
  include/${PROJECT_NAME}/flocking-application.h
)
target_link_libraries(fake_neighborhood
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
  ns3::libtap-bridge
)
ament_target_dependencies(fake_neighborhood
  Boost 
  protobuf_msgs
  rclcpp)


# add_executable(packet_success_exp src/packet_success_exp.cpp
#   include/${PROJECT_NAME}/wifi-application.cpp 
#   include/${PROJECT_NAME}/wifi-application.h
#   include/${PROJECT_NAME}/chain-flocking-application.cpp
#   include/${PROJECT_NAME}/chain-flocking-application.h
#   include/${PROJECT_NAME}/pick-neighbors-header.cpp
#   include/${PROJECT_NAME}/pick-neighbors-header.h
#   include/${PROJECT_NAME}/flocking-application.cpp
#   include/${PROJECT_NAME}/flocking-application.h
# )
# target_link_libraries(packet_success_exp
#   yaml-cpp
#   ns3::libcore
#   ns3::libwifi
#   ns3::libmobility
#   ns3::libolsr
#   ns3::libaodv
#   ns3::libdsdv
#   ns3::libdsr
#   ns3::libinternet
#   ns3::libbuildings
#   ns3::libpropagation
#   ns3::libspectrum
#   ns3::libnetwork
#   ns3::libapplications
#   ns3::libcsma
#   ns3::libtap-bridge
# )
# ament_target_dependencies(packet_success_exp
#   Boost 
#   protobuf_msgs
#   rclcpp)

message("ns-3 version " ${ns3_VERSION})

add_executable(carousel-use-case src/carousel-use-case.cpp
  include/${PROJECT_NAME}/wifi-application.cpp 
  include/${PROJECT_NAME}/wifi-application.h
  include/${PROJECT_NAME}/chain-flocking-application.cpp
  include/${PROJECT_NAME}/chain-flocking-application.h
  include/${PROJECT_NAME}/pick-neighbors-header.cpp
  include/${PROJECT_NAME}/pick-neighbors-header.h
  include/${PROJECT_NAME}/flocking-application.cpp
  include/${PROJECT_NAME}/flocking-application.h
)
target_link_libraries(carousel-use-case
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
  ns3::libtap-bridge
)
ament_target_dependencies(carousel-use-case
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
  fake_neighborhood
  # packet_success_exp
  carousel-use-case
  DESTINATION lib/${PROJECT_NAME})

ament_package()
