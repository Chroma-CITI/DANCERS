cmake_minimum_required(VERSION 3.8)
project(protobuf_msgs)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(
  ${Protobuf_INCLUDE_DIRS}
)

set(PROTO_FILES
  protobuf/physics_update.proto
  protobuf/network_update.proto
  protobuf/robots_positions.proto
)
# creates custom commands in cmake to generate protobuf code, if included in a
# target
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
protobuf_generate_python(PROTO_PY ${PROTO_FILES})

# triggers the generation of protobuf C++ code
add_library(${PROJECT_NAME}_proto ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME}_proto ${Protobuf_LIBRARIES})

# triggers the generation of protobuf python code
add_custom_target(${PROJECT_NAME}_proto_python ALL DEPENDS ${PROTO_PY})

message(STATUS "install prefix: " ${CMAKE_INSTALL_PREFIX})
message(STATUS "source dir: " ${CMAKE_SOURCE_DIR})
message(STATUS "home dir: " ${CMAKE_HOME_DIRECTORY})


# Copy protobuf generated headers from the build space to the devel space so
# that they can be used by other catkin packages. Note that the copying of
# generated protobuf headers must happen after they are built, which is only
# triggered when they are included in a target. This behavior is achived by
# setting the TARGET field of add_custom_command() appropriately.
set(PROTO_HDRS_DEVEL_DIR ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
file(MAKE_DIRECTORY ${PROTO_HDRS_DEVEL_DIR})
foreach(FIL ${PROTO_HDRS})
  add_custom_command(
    TARGET ${PROJECT_NAME}_proto
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${FIL} ${PROTO_HDRS_DEVEL_DIR}
    COMMENT "copying ${FIL} to ${PROTO_HDRS_DEVEL_DIR}"
    DEPENDS ${FIL}
    VERBATIM)
endforeach()

# # Copy protobuf generated python code from the build space to the devel space so
# # that it can be used by other catkin packages. Note that the copying of
# # generated protobuf python code must happen after it is "built", which is only
# # triggered when it is included in a target. This behavior is achived by setting
# # the TARGET field of add_custom_command() appropriately.
# set(PROTO_PYTHON_DEVEL_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
# file(MAKE_DIRECTORY ${PROTO_PYTHON_DEVEL_DIR})
# set(PYTHON_INIT_FILEPATH "${PROTO_PYTHON_DEVEL_DIR}/__init__.py")
# if(NOT EXISTS ${PYTHON_INIT_FILEPATH})
#   add_custom_target(create_empty_init ALL
#     COMMAND ${CMAKE_COMMAND} -E touch ${PYTHON_INIT_FILEPATH}
#     COMMENT "Created empty __init__.py."
#     VERBATIM)
# endif()
# foreach(FIL ${PROTO_PY})
#   add_custom_command(
#     TARGET ${PROJECT_NAME}_proto_python
#     POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy ${FIL} ${PROTO_PYTHON_DEVEL_DIR}
#     COMMENT "copying ${FIL} to ${PROTO_PYTHON_DEVEL_DIR}"
#     DEPENDS ${FIL}
#     VERBATIM)
# endforeach()

# export the targets for CMake. This is necessary to allow your library’s clients to use the target_link_libraries(client my_library::my_library) syntax. 
ament_export_targets(${PROJECT_NAME}_protoTargets HAS_LIBRARY_TARGET)
# export dependencies to downstream packages. This is necessary so that the user of the library does not have to call find_package for those dependencies, too.
ament_export_dependencies(Protobuf)

# install the header files which should be available to clients.
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_proto
  EXPORT ${PROJECT_NAME}_protoTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_proto)

ament_package()