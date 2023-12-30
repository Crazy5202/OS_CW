cmake_minimum_required(VERSION 3.10)
project(CW)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(main main.cpp)
add_executable(job job.cpp)

set(SOURCE_FILE "${CMAKE_SOURCE_DIR}/config.ini")
set(DESTINATION_DIR "${CMAKE_BINARY_DIR}")

configure_file(${SOURCE_FILE} ${DESTINATION_DIR} COPYONLY)