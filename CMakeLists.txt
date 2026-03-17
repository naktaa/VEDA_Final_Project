cmake_minimum_required(VERSION 3.16)

project(main_window VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== OpenCV =====
set(OpenCV_DIR "C:/opencv/build/x64/vc16/lib")
find_package(OpenCV REQUIRED)

# ===== Qt (Widgets + Multimedia + Network) =====
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets Network)

# ===== GStreamer (MSVC x86_64) =====
set(GST_ROOT "C:/Program Files/gstreamer/1.0/msvc_x86_64")

set(GST_INCLUDE_DIRS
    "${GST_ROOT}/include/gstreamer-1.0"
    "${GST_ROOT}/include/glib-2.0"
    "${GST_ROOT}/lib/glib-2.0/include"
)

set(GST_LIB_DIR "${GST_ROOT}/lib")

set(GST_LIBS
    "${GST_LIB_DIR}/gstreamer-1.0.lib"
    "${GST_LIB_DIR}/gstvideo-1.0.lib"
    "${GST_LIB_DIR}/gstapp-1.0.lib"
    "${GST_LIB_DIR}/gstbase-1.0.lib"
    "${GST_LIB_DIR}/gobject-2.0.lib"
    "${GST_LIB_DIR}/glib-2.0.lib"
)

# ============================================================
# ===== MQTT (vcpkg mosquitto) =====
# ============================================================
if(WIN32)
    set(VCPKG_ROOT "C:/vcpkg" CACHE PATH "vcpkg root")
    set(VCPKG_TRIPLET "x64-windows" CACHE STRING "vcpkg triplet")
    set(VCPKG_INSTALLED_DIR "${VCPKG_ROOT}/installed/${VCPKG_TRIPLET}")

    # toolchain ?�이??find_package가 ?�게 prefix 강제
    list(PREPEND CMAKE_PREFIX_PATH "${VCPKG_INSTALLED_DIR}")
endif()

# unofficial-mosquitto ?�선
find_package(unofficial-mosquitto CONFIG QUIET)
if(unofficial-mosquitto_FOUND)
    set(MOSQ_TARGET unofficial::mosquitto::mosquitto)
else()
    # fallback (?�경???�라 ?��??�공???�을 ???�어 include/lib 직접 링크??준�?
    find_package(mosquitto CONFIG QUIET)
    set(MOSQ_TARGET "")
endif()

# ===== Sources =====
set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui

    login_dialog.cpp
    login_dialog.h
    login_dialog.ui

    video_clip.cpp
    video_clip.h

    AllCctvWindow.ui
    AllCctvWindow.h
    AllCctvWindow.cpp

    CctvZoomWindow.h
    CctvZoomWindow.cpp

    GstVideoWidget.cpp
    GstVideoWidget.h

    PoseTypes.h
    PoseOverlayWidget.cpp
    PoseOverlayWidget.h
    RobotStatusWindow.cpp
    RobotStatusWindow.h
    SystemUsageWindow.cpp
    SystemUsageWindow.h

    FullscreenViewer.cpp
    FullscreenViewer.h
    CaptureCalibOverlay.cpp
    CaptureCalibOverlay.h

    TitleBarWidget.cpp
    TitleBarWidget.h
    FramelessHelper.cpp
    FramelessHelper.h

    # MQTT 붙인 ?�일??(?��? ??��?�고 ?�던 것들)
    MqttEvent.h
    MqttSubscriber.h
    MqttSubscriber.cpp
)

# ===== Target =====
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(main_window
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        ClipPopup.h
        ClipPopup.cpp
        CctvOverlayWidget.h
        HomographyCalib.h
        MapData.h
        MiniMapWidget.h
        CctvOverlayWidget.cpp
        HomographyCalib.cpp

        MiniMapWidget.cpp
        MqttPublisher.h
        MqttPublisher.cpp

        image/ui_background.jpg
    )
else()
    add_executable(main_window ${PROJECT_SOURCES})
endif()

# ===== Include =====
target_include_directories(main_window PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${GST_INCLUDE_DIRS}
)

# ===== Link =====
target_link_libraries(main_window PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Multimedia
    Qt${QT_VERSION_MAJOR}::MultimediaWidgets
    Qt${QT_VERSION_MAJOR}::Network
    ${OpenCV_LIBS}
    ${GST_LIBS}
)

# ===== mosquitto link =====
if(MOSQ_TARGET)
    target_link_libraries(main_window PRIVATE ${MOSQ_TARGET})
else()
    # ?�겟이 ???�히??경우 직접 링크
    if(WIN32)
        target_include_directories(main_window PRIVATE "${VCPKG_INSTALLED_DIR}/include")
        target_link_directories(main_window PRIVATE "${VCPKG_INSTALLED_DIR}/lib")
        target_link_libraries(main_window PRIVATE mosquitto)
    endif()
endif()

# ===== MQTT runtime DLL copy (mosquitto + pthread + openssl) =====
if(WIN32)
    add_custom_command(TARGET main_window POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${GST_ROOT}/bin/gstapp-1.0-0.dll"
            "$<TARGET_FILE_DIR:main_window>/gstapp-1.0-0.dll"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${VCPKG_INSTALLED_DIR}/bin/mosquitto.dll"
            "$<TARGET_FILE_DIR:main_window>/mosquitto.dll"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${VCPKG_INSTALLED_DIR}/bin/pthreadVC3.dll"
            "$<TARGET_FILE_DIR:main_window>/pthreadVC3.dll"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${VCPKG_INSTALLED_DIR}/bin/libssl-3-x64.dll"
            "$<TARGET_FILE_DIR:main_window>/libssl-3-x64.dll"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${VCPKG_INSTALLED_DIR}/bin/libcrypto-3-x64.dll"
            "$<TARGET_FILE_DIR:main_window>/libcrypto-3-x64.dll"
        COMMENT "Copy MQTT runtime DLLs"
    )
endif()

# ===== Bundle/Win32 options =====
if(${QT_VERSION} VERSION_LESS 6.1.0)
    set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.main_window)
endif()

set_target_properties(main_window PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS main_window
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(main_window)
endif()










target_sources(main_window PRIVATE
    HumanBoxOverlayWidget.cpp
    HumanBoxOverlayWidget.h
)
