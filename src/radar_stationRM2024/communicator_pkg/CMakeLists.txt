cmake_minimum_required(VERSION 3.0.2)
project(communicator_pkg)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  custom_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


# add_executable(receiver src/receiver.cpp)
# add_executable(sender src/sender.cpp)
add_executable(serialTest src/serialTest.cpp)

# add_dependencies(receiver custom_msgs_generate_messages)
# add_dependencies(sender custom_msgs_generate_messages)
add_dependencies(serialTest custom_msgs_generate_messages)

# target_link_libraries(receiver
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(sender
#   ${catkin_LIBRARIES}
# )
target_link_libraries(serialTest
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/communicator_main.cpp src/communicator.cpp)
add_dependencies(${PROJECT_NAME}_node custom_msgs_generate_messages)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)