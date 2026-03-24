cmake_minimum_required(VERSION 3.10)
project(tank_hybrid_eis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -DNDEBUG)

find_package(OpenCV REQUIRED COMPONENTS core imgproc video)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
  gstreamer-rtsp-server-1.0
)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)

set(COMMON_SOURCES
  src/app_config.cpp
  src/gyro_eis.cpp
  src/gst_camera_capture.cpp
  src/hybrid_eis.cpp
  src/image_utils.cpp
  src/imu_reader.cpp
  src/libcamera_capture.cpp
  src/lk_tracker.cpp
  src/mqtt_drive.cpp
  src/offset_calibrator.cpp
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
)

set(COMMON_COMPILE_OPTIONS
  ${GST_CFLAGS_OTHER}
  ${LIBCAMERA_CFLAGS_OTHER}
  ${MOSQUITTO_CFLAGS_OTHER}
)

set(COMMON_LINK_LIBS
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${LIBCAMERA_LIBRARIES}
  ${MOSQUITTO_LIBRARIES}
  Threads::Threads
  wiringPi
)

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
  src/gst_camera_capture.cpp
  src/image_utils.cpp
  src/libcamera_capture.cpp
  src/mqtt_drive.cpp
  src/rtsp_server.cpp
  src/tank_drive.cpp
  src/timebase.cpp
  src/vr_remote_input.cpp
)

target_include_directories(main_raw PRIVATE ${COMMON_INCLUDE_DIRS})
target_compile_options(main_raw PRIVATE ${COMMON_COMPILE_OPTIONS})
target_link_libraries(main_raw PRIVATE ${COMMON_LINK_LIBS})

add_executable(calib
  src/calib_offset.cpp
  src/app_config.cpp
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
  Threads::Threads
  wiringPi
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config_template.ini")
  configure_file(config_template.ini config_template.ini COPYONLY)
endif()
