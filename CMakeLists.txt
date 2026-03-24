cmake_minimum_required(VERSION 3.16)

project(VEDA_Server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQ REQUIRED libmosquitto)

add_executable(p1_tracker
    p1_tracker.cpp
)

target_include_directories(p1_tracker PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${MOSQ_INCLUDE_DIRS}
)

target_link_libraries(p1_tracker PRIVATE
    ${OpenCV_LIBS}
    ${MOSQ_LIBRARIES}
)

add_executable(homography_mqtt_pub
    calib_file/homography_mqtt_pub.cpp
)

target_include_directories(homography_mqtt_pub PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${MOSQ_INCLUDE_DIRS}
)

target_link_libraries(homography_mqtt_pub PRIVATE
    ${OpenCV_LIBS}
    ${MOSQ_LIBRARIES}
)

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
