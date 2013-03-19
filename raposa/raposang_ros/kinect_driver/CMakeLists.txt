cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


#MUDAR ISTO PARA UMA COISA MAIS COMO DEVE SER
#find_package(usb-1.0 REQUIRED) #pkg_check_modules (LIBUSB REQUIRED libusb-1.0)


# Driver auxiliar
include_directories (/usr/include/libusb-1.0) #include_directories (${LIBUSB_INCLUDE_DIRS})
rosbuild_add_executable(kinect_aux_node src/kinect_aux.cpp)
target_link_libraries (kinect_aux_node usb-1.0) #target_link_libraries (kinect_aux_node ${LIBUSB_LIBRARY_DIRS})

# Kinect OpenNI Driver
rosbuild_add_library (kinect_openni_driver_lib src/kinect_driver.cpp)                    
rosbuild_add_executable (kinect_driver_node src/kinect_driver_node.cpp)
target_link_libraries (kinect_driver_node kinect_openni_driver_lib)

# Compressor and Decompressor
rosbuild_add_executable (pointcloud_compressor_node src/pointcloud_compressor_node.cpp)
rosbuild_add_executable (pointcloud_decompressor_node src/pointcloud_decompressor_node.cpp)
