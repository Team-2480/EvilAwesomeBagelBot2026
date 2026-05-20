cmake_minimum_required(VERSION 3.31)
set(CMAKE_CXX_STANDARD 20)

project(FrcBase)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# TODO: sim support

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules")
include(CPMWrapper)
include(UseCCache)
include(maven)

add_subdirectory(src/main/cpp/)
