cmake_minimum_required(VERSION 3.10)
project(eis)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -DNDEBUG)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
  gstreamer-rtsp-server-1.0
)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
pkg_check_modules(MOSQUITTO libmosquitto)

add_executable(eis
  src/main.cpp
  src/eis_globals.cpp
  src/eis_input.cpp
  src/eis_imu.cpp
  src/eis_capture.cpp
  src/libcamera_grabber.cpp
  src/rtsp_server.cpp
  src/tank_drive.cpp
  src/gyro_eis.cpp
  src/eis_config.cpp
)

add_executable(eis_LKonly
  src/eis_LKonly.cpp
  src/tank_drive.cpp
  src/eis_config.cpp
)

add_executable(move_raw
  src/move_raw.cpp
  src/eis_globals.cpp
  src/eis_input.cpp
  src/gyro_eis.cpp
  src/libcamera_grabber.cpp
  src/rtsp_server.cpp
  src/tank_drive.cpp
  src/eis_config.cpp
)

if (MOSQUITTO_FOUND)
  add_executable(mqtt_tank_drive
    src/mqtt_tank_drive.cpp
    src/tank_drive.cpp
  )
endif()

target_include_directories(eis PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${LIBCAMERA_INCLUDE_DIRS}
)

target_compile_options(eis PRIVATE
  ${GST_CFLAGS_OTHER}
  ${LIBCAMERA_CFLAGS_OTHER}
)

target_link_libraries(eis
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${LIBCAMERA_LIBRARIES}
  wiringPi
)

target_include_directories(move_raw PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${LIBCAMERA_INCLUDE_DIRS}
)

target_compile_options(move_raw PRIVATE
  ${GST_CFLAGS_OTHER}
  ${LIBCAMERA_CFLAGS_OTHER}
)

target_link_libraries(move_raw
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${LIBCAMERA_LIBRARIES}
  wiringPi
)

if (MOSQUITTO_FOUND)
  target_include_directories(mqtt_tank_drive PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MOSQUITTO_INCLUDE_DIRS}
  )

  target_compile_options(mqtt_tank_drive PRIVATE
    ${MOSQUITTO_CFLAGS_OTHER}
  )

  target_link_libraries(mqtt_tank_drive
    ${MOSQUITTO_LIBRARIES}
    wiringPi
  )
else()
  message(WARNING "libmosquitto not found: mqtt_tank_drive target will not be built")
endif()

target_include_directories(eis_LKonly PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

target_compile_options(eis_LKonly PRIVATE
  ${GST_CFLAGS_OTHER}
)

target_link_libraries(eis_LKonly
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  wiringPi
)
