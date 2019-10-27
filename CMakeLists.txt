cmake_minimum_required(VERSION 3.12)

project(psim C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Werror -Wextra)
endif()

add_subdirectory(src)

enable_testing()
add_subdirectory(gtest)
add_subdirectory(tests)
