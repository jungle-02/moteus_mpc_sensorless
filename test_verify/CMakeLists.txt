cmake_minimum_required(VERSION 3.16)
project(test_toolchain C CXX)

message(STATUS "Verify CXX Flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "Verify Linker Flags: ${CMAKE_EXE_LINKER_FLAGS}")

add_executable(test_bin main.cpp)
