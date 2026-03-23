cmake_minimum_required(VERSION 3.16)
project(tilting_vr_server LANGUAGES CXX)

cmake_minimum_required(VERSION 3.16)
project(tilting_vr_server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(nlohmann_json QUIET)

pkg_check_modules(HTTPLIB REQUIRED cpp-httplib)

add_executable(tilting_vr_server
  src/main.cpp
)

target_include_directories(tilting_vr_server PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${HTTPLIB_INCLUDE_DIRS}
)

target_link_directories(tilting_vr_server PRIVATE
  ${HTTPLIB_LIBRARY_DIRS}
)

target_link_libraries(tilting_vr_server PRIVATE
  ${OpenCV_LIBS}
  Threads::Threads
  ${HTTPLIB_LIBRARIES}
)

if(nlohmann_json_FOUND)
  target_link_libraries(tilting_vr_server PRIVATE nlohmann_json::nlohmann_json)
endif()

# Expected headers from system packages:
# - cpp-httplib: httplib.h
