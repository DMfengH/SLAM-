cmake_minimum_required(VERSION 3.5)
project(readdepth)

set(CMAKE_CXX_STANDARD 11)
include_directories( "/usr/local/include/eigen3")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(readdd readdd.cpp)
target_link_libraries(readdd ${OpenCV_LIBS})