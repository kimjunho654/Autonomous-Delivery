cmake_minimum_required(VERSION 3.0.2)
project(testui2)

set(CMAKE_AUTOUIC ON)

find_package(
    catkin REQUIRED
    COMPONENTS
    qt_build
    roscpp
    std_msgs
    sensor_msgs
    image_transport
    cv_bridge
    tf
    geometry_msgs
)


find_package(OpenCV REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
)

catkin_package(
    CATKIN_DEPENDS tf
)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

set(QT_SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/mainwindow.ui
)

add_executable(testui2 ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP})
target_link_libraries(testui2
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
)

install(TARGETS testui2 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
