cmake_minimum_required(VERSION 3.22)

project(pvtol_gz LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(gz-sim8 REQUIRED)

add_library(pvtol_controller SHARED
  src/control/quaternion_algorithm.cc
  src/pvtol_controller.cc
  src/plugin.cc
)

target_include_directories(pvtol_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pvtol_controller PRIVATE
  gz-sim8::gz-sim8
)
