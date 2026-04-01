cmake_minimum_required(VERSION 3.16)

project(VEDA_Server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQ REQUIRED libmosquitto)

add_library(server_core
    src/server_app.cpp
    src/server_utils.cpp
    src/homography_publisher.cpp
    src/pose_tracker.cpp
)

target_include_directories(server_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${MOSQ_INCLUDE_DIRS}
)

target_compile_options(server_core PRIVATE ${MOSQ_CFLAGS_OTHER})
target_link_directories(server_core PRIVATE ${MOSQ_LIBRARY_DIRS})

target_link_libraries(server_core PUBLIC
    ${OpenCV_LIBS}
    ${MOSQ_LIBRARIES}
)

add_executable(server_main
    server_main.cpp
)

target_link_libraries(server_main PRIVATE server_core)
set_target_properties(server_main PROPERTIES OUTPUT_NAME main)

add_executable(calib_aruco_homography
    calib_file/calib_aruco_homography.cpp
)

target_include_directories(calib_aruco_homography PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(calib_aruco_homography PRIVATE
    ${OpenCV_LIBS}
)

add_executable(calib_camera
    calib_file/calib_camera.cpp
)

target_include_directories(calib_camera PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(calib_camera PRIVATE
    ${OpenCV_LIBS}
)
