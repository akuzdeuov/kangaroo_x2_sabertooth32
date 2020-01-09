cmake_minimum_required(VERSION 3.13)
project(motorControl)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

add_executable(motorControl main.cpp src/motorControl.cpp)