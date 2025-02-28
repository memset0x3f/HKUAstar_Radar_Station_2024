cmake_minimum_required(VERSION 3.0.2)
project(gui_pkg)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(OpenCV_DIR /usr/share/OpenCV/)
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    sensor_msgs
    std_msgs
    custom_msgs
)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(Qt5 REQUIRED Core Widgets)

set(QT_LIBRARIES Qt5::Widgets)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    )

catkin_package()

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/gui_pkg/*.hpp *.h)

qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(${PROJECT_NAME}_node 
    ${QT_SOURCES} 
    ${QT_RESOURCES_CPP} 
    ${QT_FORMS_HPP} 
    ${QT_MOC_HPP}
)

add_dependencies(${PROJECT_NAME}_node custom_msgs_generate_messages)

target_link_libraries(${PROJECT_NAME}_node 
    ${QT_LIBRARIES} 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(${PROJECT_NAME}_node
    /usr/lib/x86_64-linux-gnu/libtiff.so.5
    /usr/lib/x86_64-linux-gnu/libsqlite3.so.0
)
message(STATUS "OpenCV_INCLUDE_DIRS ?" : ${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_VERSION ?" : ${OpenCV_VERSION})
message(STATUS "OpenCV_LIB ?" : ${OpenCV_LIBS})
message(STATUS "OpenCV_CONGIG_PATH ?" : ${OpenCV_CONFIG_PATH})

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
