cmake_minimum_required(VERSION 3.16)

project(VEDA_RC_Auto LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQ REQUIRED libmosquitto)

find_library(WIRINGPI_LIBRARY NAMES wiringPi)

add_executable(rc_control_node
    rc/rc_control_node.cpp
    src/rc_status_types.cpp
)

target_include_directories(rc_control_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rc
    ${MOSQ_INCLUDE_DIRS}
)

target_link_libraries(rc_control_node PRIVATE
    ${MOSQ_LIBRARIES}
    Threads::Threads
)

if(WIRINGPI_LIBRARY)
    target_link_libraries(rc_control_node PRIVATE ${WIRINGPI_LIBRARY})
endif()
