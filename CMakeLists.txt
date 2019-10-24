cmake_minimum_required(VERSION 3.12)

project(PSim C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

enable_testing()
add_subdirectory(gtest)
add_subdirectory(src)
add_subdirectory(test)
