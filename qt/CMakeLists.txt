cmake_minimum_required(VERSION 3.16)

project(VEDA_QtClient VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(OpenCV_DIR "" CACHE PATH "OpenCV CMake package directory")
find_package(OpenCV REQUIRED)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets Network)

file(GLOB QT_PROJECT_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui"
)

set(QT_RESOURCE_FILES
    resources/app_resources.qrc
)

set(QT_GST_INCLUDE_DIRS "")
set(QT_GST_LINK_LIBS "")
set(QT_MOSQ_INCLUDE_DIRS "")
set(QT_MOSQ_LINK_LIBS "")
set(QT_MOSQ_COMPILE_OPTIONS "")

if(WIN32)
    set(GSTREAMER_ROOT "" CACHE PATH "GStreamer root directory")
    set(VCPKG_ROOT "" CACHE PATH "vcpkg root directory")
    set(VCPKG_TRIPLET "x64-windows" CACHE STRING "vcpkg triplet")

    if(NOT GSTREAMER_ROOT)
        message(FATAL_ERROR "Set -DGSTREAMER_ROOT=<path> to the GStreamer installation root.")
    endif()

    if(VCPKG_ROOT)
        set(VCPKG_INSTALLED_DIR "${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}")
        list(PREPEND CMAKE_PREFIX_PATH "${VCPKG_INSTALLED_DIR}")
    endif()

    set(QT_GST_INCLUDE_DIRS
        "${GSTREAMER_ROOT}/include/gstreamer-1.0"
        "${GSTREAMER_ROOT}/include/glib-2.0"
        "${GSTREAMER_ROOT}/lib/glib-2.0/include"
    )

    set(QT_GST_LIB_DIR "${GSTREAMER_ROOT}/lib")
    set(QT_GST_LINK_LIBS
        "${QT_GST_LIB_DIR}/gstreamer-1.0.lib"
        "${QT_GST_LIB_DIR}/gstvideo-1.0.lib"
        "${QT_GST_LIB_DIR}/gstapp-1.0.lib"
        "${QT_GST_LIB_DIR}/gstbase-1.0.lib"
        "${QT_GST_LIB_DIR}/gobject-2.0.lib"
        "${QT_GST_LIB_DIR}/glib-2.0.lib"
    )

    find_package(unofficial-mosquitto CONFIG QUIET)
    if(unofficial-mosquitto_FOUND)
        set(QT_MOSQ_LINK_LIBS unofficial::mosquitto::mosquitto)
    else()
        find_package(mosquitto CONFIG QUIET)
        if(VCPKG_ROOT)
            set(QT_MOSQ_INCLUDE_DIRS "${VCPKG_INSTALLED_DIR}/include")
            link_directories("${VCPKG_INSTALLED_DIR}/lib")
            list(APPEND QT_MOSQ_LINK_LIBS mosquitto)
        endif()
    endif()
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(QT_GST REQUIRED
        gstreamer-1.0
        gstreamer-app-1.0
        gstreamer-video-1.0
    )
    pkg_check_modules(QT_MOSQ REQUIRED libmosquitto)

    set(QT_GST_INCLUDE_DIRS ${QT_GST_INCLUDE_DIRS})
    set(QT_GST_LINK_LIBS ${QT_GST_LIBRARIES})
    set(QT_MOSQ_INCLUDE_DIRS ${QT_MOSQ_INCLUDE_DIRS})
    set(QT_MOSQ_LINK_LIBS ${QT_MOSQ_LIBRARIES})
    set(QT_MOSQ_COMPILE_OPTIONS ${QT_MOSQ_CFLAGS_OTHER})
endif()

if(QT_VERSION_MAJOR GREATER_EQUAL 6)
    qt_add_executable(main_window
        MANUAL_FINALIZATION
        ${QT_PROJECT_FILES}
        ${QT_RESOURCE_FILES}
    )
else()
    add_executable(main_window
        ${QT_PROJECT_FILES}
        ${QT_RESOURCE_FILES}
    )
endif()

target_include_directories(main_window PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${QT_GST_INCLUDE_DIRS}
    ${QT_MOSQ_INCLUDE_DIRS}
)

target_compile_options(main_window PRIVATE ${QT_MOSQ_COMPILE_OPTIONS})

target_link_libraries(main_window PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Multimedia
    Qt${QT_VERSION_MAJOR}::MultimediaWidgets
    Qt${QT_VERSION_MAJOR}::Network
    ${OpenCV_LIBS}
    ${QT_GST_LINK_LIBS}
    ${QT_MOSQ_LINK_LIBS}
)

add_custom_command(TARGET main_window POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:main_window>/config"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/config/runtime.ini.example"
        "$<TARGET_FILE_DIR:main_window>/config/runtime.ini.example"
    COMMENT "Copy Qt runtime configuration template"
)

if(WIN32 AND VCPKG_ROOT)
    foreach(runtime_dll
        "${GSTREAMER_ROOT}/bin/gstapp-1.0-0.dll"
        "${VCPKG_INSTALLED_DIR}/bin/mosquitto.dll"
        "${VCPKG_INSTALLED_DIR}/bin/pthreadVC3.dll"
        "${VCPKG_INSTALLED_DIR}/bin/libssl-3-x64.dll"
        "${VCPKG_INSTALLED_DIR}/bin/libcrypto-3-x64.dll"
    )
        if(EXISTS "${runtime_dll}")
            add_custom_command(TARGET main_window POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${runtime_dll}"
                    "$<TARGET_FILE_DIR:main_window>/"
            )
        endif()
    endforeach()
endif()

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(main_window)
endif()
