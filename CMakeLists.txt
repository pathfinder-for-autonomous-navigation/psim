cmake_minimum_required(VERSION 3.12)

project(psim C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

option(ENABLE_CI_BUILD "Build for CI testing" OFF)

if(ENABLE_CI_BUILD)
  add_compile_options(-DCI_BUILD)
endif()

add_compile_options(
  -Wall
  -Werror
  -Wextra
  -Wno-error=unused-parameter
)

add_subdirectory(src)

enable_testing()
add_subdirectory(gtest)
add_subdirectory(tests)
