cmake_minimum_required(VERSION 3.10)
project(eis_tank_drive)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_library(WIRINGPI_LIB wiringPi)

if(NOT WIRINGPI_LIB)
  message(FATAL_ERROR "wiringPi library not found")
endif()

add_executable(eis eis.cpp)

target_link_libraries(eis
  ${WIRINGPI_LIB}
  Threads::Threads
)
