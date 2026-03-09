cmake_minimum_required(VERSION 3.16)
project(mqtt_patrol LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)
find_package(Threads REQUIRED)
find_library(WIRINGPI_LIB wiringPi REQUIRED)

add_executable(p1_tracker p1_tracker.cpp)
target_link_libraries(
    p1_tracker
    PRIVATE
        ${OpenCV_LIBS}
        PkgConfig::MOSQUITTO
        Threads::Threads)

add_executable(rc_control_node rc_control_node.cpp)
target_link_libraries(
    rc_control_node
    PRIVATE
        PkgConfig::MOSQUITTO
        ${WIRINGPI_LIB}
        Threads::Threads)

add_executable(rc_goal_subscriber rc_goal_subscriber.cpp)
target_link_libraries(
    rc_goal_subscriber
    PRIVATE
        PkgConfig::MOSQUITTO
        Threads::Threads)
