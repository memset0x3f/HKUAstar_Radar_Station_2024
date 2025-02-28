cmake_minimum_required(VERSION 3.0.2)
project(distance_pkg)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  custom_msgs
  tf
)

find_package(OpenCV)
find_package(PCL 1.2 REQUIRED)
#find_package(libusb REQUIRED HINTS ${MVS_PATH})


catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES distance_pkg
  #  CATKIN_DEPENDS roscpp rospy std_msgs
  #  DEPENDS system_lib
)


include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_node src/distance.cpp)

add_dependencies(${PROJECT_NAME}_node custom_msgs_generate_messages)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
# project_lib
)
