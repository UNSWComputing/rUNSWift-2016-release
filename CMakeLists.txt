cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)


project(runswift)

set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")
include(cotire)

add_definitions(-Wall)
#add_definitions(-Wall -Werror)

# Keep build process generic for v[45]
message("Toolchain File: ${CMAKE_TOOLCHAIN_FILE}")
message("CTC_DIR: ${CTC_DIR}")
add_definitions(-DCROSS_COMPILING_FOR_ROBOT)
if(${CTC_DIR} MATCHES atom)
    message("Nao v4/5 atom build detected")
    add_definitions(-march=atom -mtune=atom)
else()
    message(FATAL_ERROR "Unknown CTC_DIR")
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap-eigen.cmake")

add_subdirectory(robot)

if(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Darwin")
    message("Skipping offnao build on OSX")
else()
    add_subdirectory(utils)
endif()
