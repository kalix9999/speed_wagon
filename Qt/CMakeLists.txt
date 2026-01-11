cmake_minimum_required(VERSION 3.16)

project(speed_wagon VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(OpenCV_DIR "C:/opencv/build/x64/vc16/lib") # OPENCV용

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Network)
find_package(OpenCV REQUIRED) # OPENCV용

# Tesseract 라이브러리 찾기 (CONFIG 모드가 실패할 경우 MODULE 모드 시도)
find_package(Tesseract CONFIG QUIET)
if(NOT Tesseract_FOUND)
    find_package(Tesseract MODULE REQUIRED)
endif()

# Leptonica 라이브러리 찾기 (Tesseract 의존성)
find_package(Leptonica CONFIG QUIET)
if(NOT Leptonica_FOUND)
    find_package(Leptonica MODULE REQUIRED)
    # Leptonica MODULE 모드의 경우 타겟 이름이 다를 수 있음
    if(NOT TARGET Leptonica::Leptonica AND NOT TARGET leptonica)
        # 직접 경로를 지정하거나 pkg-config 사용
        message(WARNING "Leptonica not found. Please ensure it's installed and in your PATH.")
    endif()
endif()

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(speed_wagon
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        ClientPage.h
        ClientPage.cpp
        ClientPage.ui
        testimage.h testimage.cpp testimage.ui
        tesseractocr.h tesseractocr.cpp
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET speed_wagon APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(speed_wagon SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(speed_wagon
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(speed_wagon PRIVATE 
    Qt${QT_VERSION_MAJOR}::Widgets 
    Qt${QT_VERSION_MAJOR}::Network
    ${OpenCV_LIBS}  # OPENCV용
)

# Tesseract 라이브러리 링크
if(TARGET Tesseract::libtesseract)
    target_link_libraries(speed_wagon PRIVATE Tesseract::libtesseract)
elseif(TARGET tesseract)
    target_link_libraries(speed_wagon PRIVATE tesseract)
else()
    message(FATAL_ERROR "Tesseract library target not found")
endif()

# Leptonica 라이브러리 링크
if(TARGET Leptonica::Leptonica)
    target_link_libraries(speed_wagon PRIVATE Leptonica::Leptonica)
elseif(TARGET leptonica)
    target_link_libraries(speed_wagon PRIVATE leptonica)
else()
    # Windows에서는 보통 직접 라이브러리 파일명을 사용해야 할 수 있음
    message(WARNING "Leptonica target not found. Linking may fail.")
endif()

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.speed_wagon)
endif()
set_target_properties(speed_wagon PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS speed_wagon
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(speed_wagon)
endif()
