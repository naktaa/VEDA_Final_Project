cmake_minimum_required(VERSION 3.16)
project(tilting_vr_server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json QUIET)
find_package(cpp-httplib CONFIG QUIET)

add_executable(tilting_vr_server
  src/main.cpp
)

target_include_directories(tilting_vr_server PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(tilting_vr_server PRIVATE
  ${OpenCV_LIBS}
  Threads::Threads
)

if(nlohmann_json_FOUND)
  target_link_libraries(tilting_vr_server PRIVATE nlohmann_json::nlohmann_json)
endif()

if(cpp-httplib_FOUND)
  target_link_libraries(tilting_vr_server PRIVATE cpp-httplib::cpp-httplib)
else()
  find_path(HTTPLIB_INCLUDE_DIR httplib.h)
  if(HTTPLIB_INCLUDE_DIR)
    target_include_directories(tilting_vr_server PRIVATE ${HTTPLIB_INCLUDE_DIR})
    message(STATUS "Using header-only httplib from: ${HTTPLIB_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "cpp-httplib not found. Install the package or provide httplib.h in the include path.")
  endif()
endif()
