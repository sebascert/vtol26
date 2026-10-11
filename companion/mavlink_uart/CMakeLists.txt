cmake_minimum_required(VERSION 3.10)

project(mavlink_uart)

set(CMAKE_CXX_STANDARD 17)

add_executable(
    mavlink_uart

    src/main.cpp
    src/mavlink_receiver.cpp
    src/pso_interface.cpp
    src/mavlink_sender.cpp
)

include_directories(
    ${CMAKE_SOURCE_DIR}/mavlink
    ${CMAKE_SOURCE_DIR}/mavlink/common
    ${CMAKE_SOURCE_DIR}/src
)