cmake_minimum_required(VERSION 3.12)

add_library(src
  gnc_ode.cpp
)

target_include_directories(src PUBLIC
  ${psim_SOURCE_DIR}/include
)
