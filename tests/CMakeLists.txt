cmake_minimum_required(VERSION 3.12)

add_executable(tests
  utilities_test.cpp
  ode_test.cpp
)

target_link_libraries(tests PUBLIC
  gtest_main
  src
)

add_test(
  NAME tests
  COMMAND tests
)