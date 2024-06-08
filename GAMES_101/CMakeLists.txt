cmake_minimum_required(VERSION 3.15)
project(Homework)

set(CMAKE_EXPORT_COMPILE_COMMANDS True)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

add_subdirectory(hw00)
add_subdirectory(hw01)
add_subdirectory(hw02)
add_subdirectory(hw03)
add_subdirectory(hw04)
add_subdirectory(hw05)
add_subdirectory(hw06)