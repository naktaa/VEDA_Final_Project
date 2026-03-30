cmake_minimum_required(VERSION 3.10)
project(tank_hybrid_eis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -DNDEBUG)

find_package(OpenCV REQUIRED COMPONENTS core imgproc video imgcodecs calib3d)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenSSL QUIET)
find_package(nlohmann_json QUIET)
find_package(cpp-httplib QUIET CONFIG)

pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
  gstreamer-rtsp-server-1.0
)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
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

set(COMMON_SOURCES
  src/auto_controller.cpp
  src/app_config.cpp
  src/frame_jpeg_cache.cpp
  src/gyro_eis.cpp
  src/gpio_line.cpp
  src/gst_camera_capture.cpp
  src/hybrid_eis.cpp
  src/http_vr_server.cpp
  src/image_utils.cpp
  src/imu_reader.cpp
  src/libcamera_capture.cpp
  src/lk_tracker.cpp
  src/mqtt_drive.cpp
  src/offset_calibrator.cpp
  src/ptz_control.cpp
  src/rc_json_utils.cpp
  src/rc_path_planner.cpp
  src/rc_status_publisher.cpp
  src/rc_status_types.cpp
  src/rtsp_server.cpp
  src/tank_drive.cpp
  src/timebase.cpp
  src/vr_remote_input.cpp
)

set(COMMON_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${LIBCAMERA_INCLUDE_DIRS}
  ${MOSQUITTO_INCLUDE_DIRS}
  ${LIBGPIOD_INCLUDE_DIRS}
)

if(NOT TARGET cpp-httplib::cpp-httplib)
  list(APPEND COMMON_INCLUDE_DIRS ${HTTPLIB_INCLUDE_DIR})
endif()

if(NOT nlohmann_json_FOUND)
  list(APPEND COMMON_INCLUDE_DIRS ${NLOHMANN_JSON_INCLUDE_DIR})
endif()

set(COMMON_COMPILE_OPTIONS
  ${GST_CFLAGS_OTHER}
  ${LIBCAMERA_CFLAGS_OTHER}
  ${MOSQUITTO_CFLAGS_OTHER}
  ${LIBGPIOD_CFLAGS_OTHER}
)

if(HTTPLIB_CFLAGS_OTHER)
  list(APPEND COMMON_COMPILE_OPTIONS ${HTTPLIB_CFLAGS_OTHER})
endif()

set(COMMON_LINK_LIBS
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${LIBCAMERA_LIBRARIES}
  ${MOSQUITTO_LIBRARIES}
  ${LIBGPIOD_LIBRARIES}
  Threads::Threads
  wiringPi
)

if(TARGET cpp-httplib::cpp-httplib)
  list(APPEND COMMON_LINK_LIBS cpp-httplib::cpp-httplib)
elseif(HTTPLIB_LIBRARY)
  list(APPEND COMMON_LINK_LIBS ${HTTPLIB_LIBRARY})
endif()

if(OpenSSL_FOUND)
  add_compile_definitions(CPPHTTPLIB_OPENSSL_SUPPORT)
  list(APPEND COMMON_LINK_LIBS OpenSSL::SSL OpenSSL::Crypto)
endif()

if(nlohmann_json_FOUND)
  list(APPEND COMMON_LINK_LIBS nlohmann_json::nlohmann_json)
endif()

if(LIBGPIOD_VERSION VERSION_GREATER_EQUAL 2.0.0)
  add_compile_definitions(USE_LIBGPIOD_V2)
endif()

add_executable(main
  src/main.cpp
  ${COMMON_SOURCES}
)

target_include_directories(main PRIVATE ${COMMON_INCLUDE_DIRS})
target_compile_options(main PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(main PRIVATE ${COMMON_LINK_LIBS})

add_executable(main_raw
  src/main_raw.cpp
  src/app_config.cpp
  src/frame_jpeg_cache.cpp
  src/gst_camera_capture.cpp
  src/http_vr_server.cpp
  src/image_utils.cpp
  src/libcamera_capture.cpp
  src/mqtt_drive.cpp
  src/ptz_control.cpp
  src/rtsp_server.cpp
  src/tank_drive.cpp
  src/timebase.cpp
  src/vr_remote_input.cpp
)

target_include_directories(main_raw PRIVATE ${COMMON_INCLUDE_DIRS})
target_compile_options(main_raw PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(main_raw PRIVATE ${COMMON_LINK_LIBS})

add_executable(lkonly
  src/main_lkonly.cpp
  ${COMMON_SOURCES}
)

target_include_directories(lkonly PRIVATE ${COMMON_INCLUDE_DIRS})
target_compile_options(lkonly PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(lkonly PRIVATE ${COMMON_LINK_LIBS})

add_executable(calib
  src/calib_offset.cpp
  src/app_config.cpp
  src/gpio_line.cpp
  src/gyro_eis.cpp
  src/image_utils.cpp
  src/imu_reader.cpp
  src/libcamera_capture.cpp
  src/lk_tracker.cpp
  src/offset_calibrator.cpp
  src/timebase.cpp
)

target_include_directories(calib PRIVATE ${COMMON_INCLUDE_DIRS})
target_compile_options(calib PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(calib PRIVATE
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${LIBCAMERA_LIBRARIES}
  ${LIBGPIOD_LIBRARIES}
  Threads::Threads
  wiringPi
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config_template.ini")
  configure_file(config_template.ini config_template.ini COPYONLY)
endif()
