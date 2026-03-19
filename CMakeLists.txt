cmake_minimum_required(VERSION 3.10)
project(tank_mqtt_drive)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -DNDEBUG)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio)

add_executable(mqtt_tank_drive
  src/mqtt_tank_drive.cpp
  src/mqtt_drive.cpp
  src/camera_capture.cpp
  src/rtsp_stream.cpp
  src/tank_drive.cpp
  src/eis_processor.cpp
)

target_include_directories(mqtt_tank_drive PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MOSQUITTO_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_options(mqtt_tank_drive PRIVATE
  ${MOSQUITTO_CFLAGS_OTHER}
  ${GST_CFLAGS_OTHER}
)

target_link_libraries(mqtt_tank_drive
  ${MOSQUITTO_LIBRARIES}
  ${GST_LIBRARIES}
  ${OpenCV_LIBS}
  wiringPi
)

# 설정 파일이 빌드 디렉토리에 없으면 복사하여 로컬 튜닝 환경 구축
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config_lk_local.ini")
  configure_file(config_lk_local.ini config_lk_local.ini COPYONLY)
endif()
