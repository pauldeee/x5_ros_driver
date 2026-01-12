cmake_minimum_required(VERSION 3.0.2)
project(x5_ros_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
  image_transport
)

## Find OpenCV
find_package(OpenCV REQUIRED)

## Find FFmpeg
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED
  libavcodec
  libavformat
  libswscale
  libavutil
)

## Find libusb
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

## Insta360 SDK path
set(INSTA360_SDK_PATH "" CACHE PATH "Path to Insta360 CameraSDK")

if(NOT INSTA360_SDK_PATH)
  # Try common locations
  if(EXISTS "$ENV{HOME}/CameraSDK")
    set(INSTA360_SDK_PATH "$ENV{HOME}/CameraSDK")
  elseif(EXISTS "/opt/insta360/CameraSDK")
    set(INSTA360_SDK_PATH "/opt/insta360/CameraSDK")
  endif()
endif()

if(NOT EXISTS "${INSTA360_SDK_PATH}/include/camera/camera.h")
  message(FATAL_ERROR 
    "Insta360 CameraSDK not found!\n"
    "Please set INSTA360_SDK_PATH:\n"
    "  catkin build x5_ros_driver -DINSTA360_SDK_PATH=/path/to/CameraSDK"
  )
endif()

message(STATUS "Using Insta360 SDK at: ${INSTA360_SDK_PATH}")

## Find SDK library
find_library(INSTA360_CAMERA_LIB 
  NAMES CameraSDK libCameraSDK
  PATHS 
    "${INSTA360_SDK_PATH}/lib"
    "${INSTA360_SDK_PATH}/lib/x64"
  NO_DEFAULT_PATH
)

if(NOT INSTA360_CAMERA_LIB)
  message(FATAL_ERROR "CameraSDK library not found in ${INSTA360_SDK_PATH}/lib")
endif()

message(STATUS "Using CameraSDK library: ${INSTA360_CAMERA_LIB}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs std_srvs cv_bridge image_transport
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${LIBAV_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
  ${INSTA360_SDK_PATH}/include
)

## Library
add_library(${PROJECT_NAME}
  src/x5_camera.cpp
  src/video_decoder.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LIBAV_LIBRARIES}
  ${LIBUSB_LIBRARIES}
  ${INSTA360_CAMERA_LIB}
)

## Node executable
add_executable(x5_driver_node src/x5_driver_node.cpp)

target_link_libraries(x5_driver_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} x5_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
