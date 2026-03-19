cmake_minimum_required(VERSION 3.10)
project(tank_mqtt_drive)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -DNDEBUG)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
  gstreamer-rtsp-server-1.0
)

add_executable(mqtt_tank_drive
  src/mqtt_tank_drive.cpp
  src/mqtt_drive.cpp
  src/camera_capture.cpp
  src/rtsp_stream.cpp
  src/tank_drive.cpp
)

target_include_directories(mqtt_tank_drive PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MOSQUITTO_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

target_compile_options(mqtt_tank_drive PRIVATE
  ${MOSQUITTO_CFLAGS_OTHER}
  ${GST_CFLAGS_OTHER}
)

target_link_libraries(mqtt_tank_drive
  ${MOSQUITTO_LIBRARIES}
  ${GST_LIBRARIES}
  wiringPi
)
