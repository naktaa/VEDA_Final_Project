cmake_minimum_required(VERSION 3.13)
project(scservo_wasd CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(USE_SMS_STS "Use SMS_STS series (OFF uses SCSCL)" ON)

# Path to SCServo_Linux (override with -DSCSERVO_ROOT=/path/to/SCServo_Linux)
set(SCSERVO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../SCServo_Linux" CACHE PATH "Path to SCServo_Linux")

if(NOT EXISTS "${SCSERVO_ROOT}/SCS.cpp")
    message(FATAL_ERROR "SCServo_Linux not found. Set -DSCSERVO_ROOT=/path/to/SCServo_Linux (missing: ${SCSERVO_ROOT}/SCS.cpp)")
endif()

add_executable(scservo_wasd
    scservo_wasd.cpp
    "${SCSERVO_ROOT}/SCS.cpp"
    "${SCSERVO_ROOT}/SCSerial.cpp"
)

if(USE_SMS_STS)
    target_compile_definitions(scservo_wasd PRIVATE USE_SMS_STS=1)
    target_sources(scservo_wasd PRIVATE "${SCSERVO_ROOT}/SMS_STS.cpp")
else()
    target_compile_definitions(scservo_wasd PRIVATE USE_SMS_STS=0)
    target_sources(scservo_wasd PRIVATE "${SCSERVO_ROOT}/SCSCL.cpp")
endif()

target_include_directories(scservo_wasd PRIVATE "${SCSERVO_ROOT}")
