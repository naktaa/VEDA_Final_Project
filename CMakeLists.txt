cmake_minimum_required(VERSION 3.10)
project(tank_mqtt_drive)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -DNDEBUG)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(nlohmann_json QUIET)
find_package(cpp-httplib QUIET CONFIG)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
  gstreamer-rtsp-server-1.0
)
pkg_check_modules(HTTPLIB QUIET cpp-httplib)

if(NOT TARGET cpp-httplib::cpp-httplib)
  find_path(HTTPLIB_INCLUDE_DIR
    NAMES httplib.h
    HINTS ${HTTPLIB_INCLUDE_DIRS}
  )

  if(NOT HTTPLIB_INCLUDE_DIR)
    message(FATAL_ERROR "httplib.h not found. Install cpp-httplib development headers.")
  endif()

  find_library(HTTPLIB_LIBRARY
    NAMES cpp-httplib httplib
    HINTS ${HTTPLIB_LIBRARY_DIRS}
  )
endif()

if(NOT nlohmann_json_FOUND)
  find_path(NLOHMANN_JSON_INCLUDE_DIR NAMES nlohmann/json.hpp)
  if(NOT NLOHMANN_JSON_INCLUDE_DIR)
    message(FATAL_ERROR "nlohmann/json.hpp not found. Install nlohmann-json3-dev.")
  endif()
endif()

add_executable(mqtt_tank_drive
  src/mqtt_tank_drive.cpp
  src/frame_jpeg_cache.cpp
  src/http_vr_server.cpp
  src/mqtt_drive.cpp
  src/camera_capture.cpp
  src/ptz_control.cpp
  src/rtsp_stream.cpp
  src/tank_drive.cpp
  src/vr_remote_input.cpp
)

target_include_directories(mqtt_tank_drive PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MOSQUITTO_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

if(NOT TARGET cpp-httplib::cpp-httplib)
  target_include_directories(mqtt_tank_drive PRIVATE ${HTTPLIB_INCLUDE_DIR})
endif()

if(NOT nlohmann_json_FOUND)
  target_include_directories(mqtt_tank_drive PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR})
endif()

target_compile_options(mqtt_tank_drive PRIVATE
  ${HTTPLIB_CFLAGS_OTHER}
  ${MOSQUITTO_CFLAGS_OTHER}
  ${GST_CFLAGS_OTHER}
)

target_compile_definitions(mqtt_tank_drive PRIVATE
  TANK_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

set_target_properties(mqtt_tank_drive PROPERTIES OUTPUT_NAME main)

target_link_libraries(mqtt_tank_drive
  ${MOSQUITTO_LIBRARIES}
  ${GST_LIBRARIES}
  ${OpenCV_LIBS}
  Threads::Threads
  wiringPi
)

if(TARGET cpp-httplib::cpp-httplib)
  target_link_libraries(mqtt_tank_drive cpp-httplib::cpp-httplib)
elseif(HTTPLIB_LIBRARY)
  target_link_libraries(mqtt_tank_drive ${HTTPLIB_LIBRARY})
endif()

if(nlohmann_json_FOUND)
  target_link_libraries(mqtt_tank_drive nlohmann_json::nlohmann_json)
endif()

add_custom_command(TARGET mqtt_tank_drive POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:mqtt_tank_drive>
          $<TARGET_FILE_DIR:mqtt_tank_drive>/mqtt_tank_drive
)
