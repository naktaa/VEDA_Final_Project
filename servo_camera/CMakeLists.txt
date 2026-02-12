# CMake 프로그램의 최소 버전
cmake_minimum_required(VERSION 3.16)

# 프로젝트 정보
project(
  ModooCode
  VERSION 0.1
  DESCRIPTION "밥차 프로젝트"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 쓰레드
find_package(Threads REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# ---- mqtt 동적 라이브러리(.so) 생성 ----
add_library(mqtt_client_shared SHARED
    lib/mqtt_client.cpp
)

# mqtt_client.cpp가 mqtt_client.h를 찾게 해줌
target_include_directories(mqtt_client_shared PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/includes
)

# mqtt 라이브러리가 내부적으로 쓰는 의존성 링크
target_link_libraries(mqtt_client_shared PUBLIC
  mosquitto
  Threads::Threads
)





target_link_libraries(mqtt_client_shared PUBLIC mosquitto Threads::Threads)



#실행파일 
add_executable(rice_car
    main.cpp
)

# main.cpp가 main.h / mqtt_client.h를 찾게 해줌
target_include_directories(rice_car PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/includes)



# 실행파일 링크 (OpenCV + wiringPi + 우리가 만든 .so)
target_link_libraries(rice_car PRIVATE
  ${OpenCV_LIBS}
  wiringPi
  mqtt_client_shared
)
# (선택) 경고
target_compile_options(rice_car PRIVATE -Wall -Wextra)