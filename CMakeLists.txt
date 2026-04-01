cmake_minimum_required(VERSION 3.16)

project(VEDA_Final_Project LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ProjectOptions.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Dependencies.cmake)

veda_set_default_project_options()

option(BUILD_SERVER "Build the server module" ON)
option(BUILD_TANK "Build the tank module" ON)

if(BUILD_SERVER)
    add_subdirectory(server)
endif()

if(BUILD_TANK)
    add_subdirectory(tank)
endif()

message(STATUS "Qt client is built separately: cmake -S qt -B build/qt ...")
message(STATUS "RuView firmware is built separately: idf.py -C ruview build")
