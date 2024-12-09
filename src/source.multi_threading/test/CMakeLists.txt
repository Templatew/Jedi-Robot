#
#
#
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(PRJNAME unit_test)
message(STATUS "==TEST PROJECT: ${PRJNAME}, SOURCEDIR " ${CMAKE_SOURCE_DIR})

#
# create a library with the source file
#
project(testlib)
set(MAIN_SRC
  ${CMAKE_SOURCE_DIR}/src/glm_util.h
  ${CMAKE_SOURCE_DIR}/src/glm_util.cpp
)
add_library(testlib STATIC ${MAIN_SRC})


#
# executable
#

project(${PRJNAME} VERSION 0.1 LANGUAGES CXX)
enable_testing()
find_package(GTest CONFIG REQUIRED)


add_executable(
  ${PRJNAME}
  test_inverse_xyzrpy.cpp
)
target_include_directories(${PRJNAME} PUBLIC ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(
  ${PRJNAME}
  GTest::gtest_main
  testlib
)

include(GoogleTest)
gtest_discover_tests(${PRJNAME})
