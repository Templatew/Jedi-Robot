## header-start
################################################################################
##
## \file      CMakeList.txt (src folder)
## \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
##
################################################################################
## header-end
##

set(PROJECT_SOURCES
  logger.h
  context.h
  util.h
  bbox3D.h
  ikparams.h
  glm_util.h  glm_util.cpp
  facet.h  facet.cpp
  stlmesh.h  stlmesh.cpp
  joint.h  joint.cpp
  link.h link.cpp
  poses.h poses.cpp
  robot.h  robot.cpp
  main.cpp
)

#if(CMAKE_BUILD_TYPE STREQUAL "Sanitize")
#  add_compile_options(-g -fsanitize=leak)
#  add_link_options(-fsanitize=leak)
#endif()

add_executable(${PRJNAME} ${PROJECT_SOURCES})
target_compile_definitions(${PRJNAME} PUBLIC PROG_VERSION="${PRJVERSION}")
target_compile_definitions(${PRJNAME} PUBLIC PROG_NAME="${PRJNAME}")
target_compile_definitions(${PRJNAME} PUBLIC PROG_GITID="${PRJFULLID}")
#target_include_directories(${PRJNAME PRIVATE /absolute/path

#if(CMAKE_BUILD_TYPE STREQUAL "Sanitize")
#  set_target_properties(${PRJNAME} PROPERTIES CMAKE_SANITIZE_POSTFIX ${CMAKE_SANITIZE_POSTFIX})
#endif()

#if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#  set_target_properties(${PRJNAME} PROPERTIES CMAKE_DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
#endif()

#set_target_properties(${PRJNAME} PROPERTIES CMAKE_DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
#set_target_properties(${PRJNAME} PROPERTIES CMAKE_SANITIZE_POSTFIX ${CMAKE_SANITIZE_POSTFIX})
target_link_libraries(${PRJNAME} glm::glm)
target_link_libraries(${PRJNAME} toml11::toml11)
target_link_libraries(${PRJNAME} fmt)


