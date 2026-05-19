cmake_minimum_required(VERSION 4.0)
set(CMAKE_CXX_STANDARD 20)

project(FrcBase)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

include(maven.cmake)
add_subdirectory(src/main/cpp/)
