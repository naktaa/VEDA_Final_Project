cmake_minimum_required(VERSION 3.16)

project(VEDA_RC_Auto LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQ REQUIRED libmosquitto)

find_library(WIRINGPI_LIBRARY NAMES wiringPi)

add_library(auto_core
    src/auto_app.cpp
    src/rc_config.cpp
    src/rc_control_node.cpp
    src/rc_json_utils.cpp
    src/rc_motor_driver.cpp
    src/rc_path_planner.cpp
    src/rc_status_types.cpp
)

target_include_directories(auto_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MOSQ_INCLUDE_DIRS}
)

target_compile_options(auto_core PRIVATE ${MOSQ_CFLAGS_OTHER})
target_link_directories(auto_core PRIVATE ${MOSQ_LIBRARY_DIRS})

target_link_libraries(auto_core PUBLIC
    ${MOSQ_LIBRARIES}
    Threads::Threads
)

if(WIRINGPI_LIBRARY)
    target_link_libraries(auto_core PUBLIC ${WIRINGPI_LIBRARY})
endif()

add_executable(auto_main
    auto_main.cpp
)

target_link_libraries(auto_main PRIVATE auto_core)
set_target_properties(auto_main PROPERTIES OUTPUT_NAME main)
