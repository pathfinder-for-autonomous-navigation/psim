cmake_minimum_required(VERSION 3.12)

add_executable(tests
  environment_test.cpp
  ode_test.cpp
  utilities_test.cpp
)

target_link_libraries(tests PUBLIC
  gtest_main
  src
)

add_test(
  NAME tests
  COMMAND tests
)
