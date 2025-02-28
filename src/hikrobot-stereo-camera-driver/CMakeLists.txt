cmake_minimum_required(VERSION 3.11)

project(hikrobot_camera)

set(allowableBuildTypes Debug Release Record)

if(NOT CMAKE_BUILD_TYPE IN_LIST allowableBuildTypes)
	message(FATAL_ERROR "${CMAKE_BUILD_TYPE} is not a defined build type")
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(DEBUG)
elseif (CMAKE_BUILD_TYPE STREQUAL "Record")
  add_compile_definitions(RECORD)
# elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
#   add_compile_definitions(RELEASE)
endif()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O2")

enable_language(CUDA)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  custom_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

catkin_package(
 
)

# Find OpenCV
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4/")
find_package(OpenCV 4.2.0 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}"
    " (include: ${OpenCV_INCLUDE_DIRS}"
    " lib: ${OpenCV_LIBS})")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  include/BoTSORT
  /opt/MVS/include/
)

# Find CUDA
find_package(CUDA REQUIRED)
message(STATUS "CUDA Version: ${CUDA_VERSION}")
message(STATUS "CUDA Libs: ${CUDA_LIBRARIES}")
include_directories(${CUDA_INCLUDE_DIRS})

# Find OpenMP
find_package(OpenMP)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# Find and link Boost
find_package(Boost REQUIRED COMPONENTS filesystem)
include_directories(${Boost_INCLUDE_DIRS})

LINK_DIRECTORIES(
/opt/MVS/lib/64/
)

file(GLOB_RECURSE SOURCES "src/BoTSORT/*.cpp")
add_executable(${PROJECT_NAME} src/hikrobot_camera.cpp src/TrtEngine.cpp src/cuda/cuda_preprocess.cu ${SOURCES})
add_executable(${PROJECT_NAME}_image src/hikrobot_camera_image_write.cpp ${SOURCES})
add_executable(${PROJECT_NAME}_video src/hikrobot_camera_debug.cpp ${SOURCES})

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${CUDA_LIBRARIES}
   ${Boost_LIBRARIES}
   nvinfer
   nvonnxparser
   # GCBase_gcc421_v3_0
   MvCameraControl
   # GenApi_gcc421_v3_0
   MVGigEVisionSDK
   # log4cpp_gcc421_v3_0
   MVRender
   # Log_gcc421_v3_0
   MvUsb3vTL
   # MathParser_gcc421_v3_0
   # NodeMapData_gcc421_v3_0
   MediaProcess
   # XmlParser_gcc421_v3_0
   X11
   Eigen3::Eigen
 )

 target_link_libraries(${PROJECT_NAME}_image
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${CUDA_LIBRARIES}
   ${Boost_LIBRARIES}
   nvinfer
   nvonnxparser
   # GCBase_gcc421_v3_0
   MvCameraControl
   # GenApi_gcc421_v3_0
   MVGigEVisionSDK
   # log4cpp_gcc421_v3_0
   MVRender
   # Log_gcc421_v3_0
   MvUsb3vTL
   # MathParser_gcc421_v3_0
   # NodeMapData_gcc421_v3_0
   MediaProcess
   # XmlParser_gcc421_v3_0
   X11
   Eigen3::Eigen
 )

 target_link_libraries(${PROJECT_NAME}_video
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
 ${CUDA_LIBRARIES}
 ${Boost_LIBRARIES}
 nvinfer
 nvonnxparser
 # GCBase_gcc421_v3_0
 MvCameraControl
 # GenApi_gcc421_v3_0
 MVGigEVisionSDK
 # log4cpp_gcc421_v3_0
 MVRender
 # Log_gcc421_v3_0
 MvUsb3vTL
 # MathParser_gcc421_v3_0
 # NodeMapData_gcc421_v3_0
 MediaProcess
 # XmlParser_gcc421_v3_0
 X11
 Eigen3::Eigen
)