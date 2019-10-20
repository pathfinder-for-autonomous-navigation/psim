cmake_minimum_required(VERSION 3.12)

set(This PSim_tests)

set(Sources
  utilities_test.cpp
)

include_directories(${PSim_SOURCE_DIR}/lin/include)
include_directories(${PSim_SOURCE_DIR}/include)

add_executable(${This} ${Sources})
target_link_libraries(${This} PUBLIC
  gtest_main
)

add_test(
  NAME ${This}
  COMMAND ${This}
)
