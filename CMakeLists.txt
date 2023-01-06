cmake_minimum_required(VERSION 2.8)
project("BizerbaLelyIntegration")

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/include-lely-core.cmake)

set(CMAKE_CXX_FLAGS "-O3 -Wall -Wno-unknown-pragmas -std=c++11")

add_subdirectory(LelyIntegration)
add_subdirectory(LelyTest)
