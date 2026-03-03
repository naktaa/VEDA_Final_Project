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

find_library(WIRINGPI_LIB wiringPi)

add_executable(eis eis.cpp)

target_include_directories(eis PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

target_compile_options(eis PRIVATE
  ${GST_CFLAGS_OTHER}
)

target_link_libraries(eis
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${WIRINGPI_LIB}
)
