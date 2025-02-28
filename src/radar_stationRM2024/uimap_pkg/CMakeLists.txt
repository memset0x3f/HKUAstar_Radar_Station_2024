cmake_minimum_required(VERSION 3.0.2)
project(uimap_pkg)

find_package(catkin REQUIRED COMPONENTS
  custom_msgs
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV 4.2.0 REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${OpenCV_INCLUDE_DIRS}
)

add_compile_options(
  -std=c++17
)

add_executable(${PROJECT_NAME}_node
  src/uimap.cpp
)

add_dependencies(${PROJECT_NAME}_node
  custom_msgs_generate_messages
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_definitions(-DPACK_PATH="${PROJECT_SOURCE_DIR}")