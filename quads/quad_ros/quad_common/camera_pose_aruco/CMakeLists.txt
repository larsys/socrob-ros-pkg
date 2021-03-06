cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set( CMAKE_VERBOSE_MAKEFILE on )
project(camera_pose_aruco)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
set(CMAKE_BUILD_TYPE Coverage)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

# OpenCV
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_libraries(${OpenCV_LIBRARIES})

#aruco
#add_library(aruco SHARED IMPORTED)
#set_property(TARGET aruco PROPERTY
#                IMPORTED_LOCATION /usr/local/lib/libaruco.so)


rosbuild_add_library(aruco
   src/aruco/aruco.h
   src/aruco/arucofidmarkers.cpp
   src/aruco/arucofidmarkers.h
   src/aruco/board.cpp
   src/aruco/board.h
   src/aruco/boarddetector.cpp
   src/aruco/markerdetector.h
   src/aruco/markerdetector.cpp
   src/aruco/cvdrawingutils.cpp
   src/aruco/cameraparameters.cpp
   src/aruco/marker.cpp
   src/aruco/boarddetector.h
   src/aruco/cvdrawingutils.h
   src/aruco/exports.h
   src/aruco/cameraparameters.h
   src/aruco/marker.h)  

rosbuild_add_library(camera_pose_aruco
   src/main.cpp
   src/aruco/aruco.h
   src/aruco/cvdrawingutils.h
   src/MarkersConfig.h
   src/MarkersConfig.cpp
)

target_link_libraries(camera_pose_aruco
    aruco)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#rosbuild_add_library(libs src/aruco/aruco.h)
#target_link_libraries(${PROJECT_NAME} another_library)
#target_link_libraries(libs)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(camera_pose_aruco src/main.cpp)
#target_link_libraries(aruco camera_pose_aruco_lib)
