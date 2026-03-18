cmake_minimum_required(VERSION 3.16)

project(VEDA_RC_Status_Publisher LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(MOSQUITTO QUIET IMPORTED_TARGET libmosquitto)
endif()

set(MOSQUITTO_FOUND_BY_FALLBACK OFF)
if(NOT TARGET PkgConfig::MOSQUITTO)
    find_path(MOSQUITTO_INCLUDE_DIR mosquitto.h)
    find_library(MOSQUITTO_LIBRARY NAMES mosquitto)

    if(MOSQUITTO_INCLUDE_DIR AND MOSQUITTO_LIBRARY)
        set(MOSQUITTO_FOUND_BY_FALLBACK ON)
    else()
        message(FATAL_ERROR "libmosquitto not found. Install libmosquitto-dev (or equivalent).")
    endif()
endif()

find_package(nlohmann_json QUIET)

add_library(rc_status_publisher_core
    src/rc_status_types.cpp
    src/rc_status_publisher.cpp
)

target_include_directories(rc_status_publisher_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MOSQUITTO_FOUND_BY_FALLBACK)
    target_include_directories(rc_status_publisher_core PUBLIC ${MOSQUITTO_INCLUDE_DIR})
    target_link_libraries(rc_status_publisher_core PUBLIC ${MOSQUITTO_LIBRARY})
else()
    target_link_libraries(rc_status_publisher_core PUBLIC PkgConfig::MOSQUITTO)
endif()

target_link_libraries(rc_status_publisher_core PUBLIC Threads::Threads)

if(TARGET nlohmann_json::nlohmann_json)
    target_link_libraries(rc_status_publisher_core PUBLIC nlohmann_json::nlohmann_json)
endif()

add_executable(rc_status_publisher_app
    src/rc_status_publisher_main.cpp
)

target_link_libraries(rc_status_publisher_app PRIVATE rc_status_publisher_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(rc_status_publisher_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(rc_status_publisher_app PRIVATE -Wall -Wextra -Wpedantic)
endif()
