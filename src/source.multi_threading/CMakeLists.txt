## header-start
################################################################################
##
## \file      CMakeList.txt (base folder)
## \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
##
################################################################################
## header-end
##

cmake_minimum_required(VERSION 3.22)

set(PRJNAME robot_kinematic)
set(PRJVERSION 1.0)

find_package(Git REQUIRED)
execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --short=12 HEAD OUTPUT_VARIABLE COMMITID OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND "${GIT_EXECUTABLE}" status --short COMMAND fgrep -c " M " OUTPUT_VARIABLE GITCHANGEDFILE OUTPUT_STRIP_TRAILING_WHITESPACE)
 
set(PRJFULLID "${COMMITID}+${GITCHANGEDFILE}")

project(
    ${PRJNAME}
    VERSION ${PRJVERSION}
    DESCRIPTION "Computing FK and IK for robotic arms"
    LANGUAGES CXX)
    message(STATUS "Project: ${PRJNAME}, Version: ${PRJVERSION}, CommitId: ${PRJFULLID}"
)

include(FetchContent)

FetchContent_Declare(
  glm
  GIT_REPOSITORY	https://github.com/g-truc/glm.git
  GIT_TAG 	bf71a834948186f4097caa076cd2663c69a10e1e #refs/tags/1.0.1
)

# Add optimization flag for fastest optimization
add_compile_options(-O3)
message(STATUS "Fetching glm library")
FetchContent_MakeAvailable(glm)

FetchContent_Declare(fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG master
)
message(STATUS "Fetching fmt library")
FetchContent_MakeAvailable(fmt)

#include(FetchContent)
#FetchContent_Declare(
#    tomlplusplus
#    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
#    GIT_TAG v3.4.0
#)
#FetchContent_MakeAvailable(tomlplusplus)

FetchContent_Declare(
  toml11
  GIT_REPOSITORY https://github.com/ToruNiina/toml11.git
  GIT_TAG        v4.1.0
)
message(STATUS "Fetching toml11 library")
FetchContent_MakeAvailable(toml11)


add_compile_options(-O3)

set(CMAKE_DEBUG_POSTFIX .d)
set(CMAKE_SANITIZE_POSTFIX .s)

#if(CMAKE_BUILD_TYPE STREQUAL "Sanitize")
#  message("Sanitize: Setting the debug postfix to .s")
#endif()


set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_INCLUDE_CURRENT_DIR ON)


add_compile_options(-Wall -Wextra -Wno-unused-parameter -Werror)

#  add_compile_options(-g -fsanitize=leak)
#  add_link_options(-fsanitize=leak)

SET(CMAKE_CXX_FLAGS_SANITIZE
    "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=leak -fno-omit-frame-pointer"
    CACHE STRING "Flags used by the C++ compiler for leak sanitized build type or configuration."
    FORCE)

SET(CMAKE_EXE_LINKER_FLAGS_SANITIZE
    "-fsanitize=leak"
    CACHE STRING "Flags used for linking binaries during sanitize builds."
    FORCE )

MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_SANITIZE
    CMAKE_EXE_LINKER_FLAGS_SANITIZE )



#CMAKE_<CONFIG>_POSTFIX: Adds a custom "postfix" to static and
#shared libraries when in a certain build type. Example: Setting



#find_package(glm CONFIG REQUIRED)
add_subdirectory(src)

message(STATUS "Current build type is : ${CMAKE_BUILD_TYPE}")


if (ENABLE_TEST)
  enable_testing()
  add_subdirectory(test)
endif ()

#include(src)

#add_executable(${PRJNAME} ${PROJECT_SOURCES})
#set_target_properties(${PRJNAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
#target_link_libraries(${PRJNAME} glm::glm)

#target_compile_definitions(${PRJNAME} PUBLIC VERSION=${PRJVERSION})
#set_target_properties(${PRJNAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
#target_link_libraries(${PRJNAME} glm::glm)

#### additional help on CMakeLists.txt
#
# refer to https://gitlab.com/CLIUtils/modern-cmake/-/tree/master/examples/extended-project 
# for more details on this CMAKE
#

