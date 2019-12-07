cmake_minimum_required(VERSION 3.09)

# set the project name
project(TestGeoGrav)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package (GeographicLib REQUIRED)
# add the executable
add_executable(TestGeoGrav main.cpp)
target_link_libraries (TestGeoGrav ${GeographicLib_LIBRARIES})
