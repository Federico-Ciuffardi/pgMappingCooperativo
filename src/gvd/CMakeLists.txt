cmake_minimum_required(VERSION 3.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
project(main)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Find sources
file(GLOB MAIN_SOURCES
  ./*.h
  ./*.cpp
)

# Set include paths
include_directories(GLOB MAIN_INCLUDES
  .
)

add_executable(test ${MAIN_SOURCES})

target_include_directories(test PRIVATE ${MAIN_INCLUDES})