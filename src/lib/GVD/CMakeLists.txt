cmake_minimum_required(VERSION 3.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
project(main)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Find sources
file(GLOB MAIN_SOURCES
  ./src/data/*.cpp
  ./src/data/*.cpp
  ./src/*.h
  ./src/*.cpp
  ./*.h
  ./*.cpp
)

# Set include paths
include_directories(GLOB MAIN_INCLUDES
  .
)

add_executable(tests ${MAIN_SOURCES})

target_include_directories(tests PRIVATE ${MAIN_INCLUDES})
