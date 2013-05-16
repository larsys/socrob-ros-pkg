cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(LIBUSB_INCLUDE_PATH /usr/include/libusb-1.0/)
set(LIBUSB_NAME 				usb-1.0)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(libz800 libz800math/Matrixd.cpp libz800math/Utils.cpp libz800math/Matrixf.cpp libz800math/Quat.cpp 
														 libz800/Calc.cpp libz800/Device.cpp libz800/libz800.cpp 
														 libz800/Property.cpp libz800/QuatKalmanFilter.cpp libz800/Timer.cpp)
include_directories(libz800math libz800 ${LIBUSB_INCLUDE_PATH})
target_link_libraries(libz800 ${LIBUSB_NAME})

rosbuild_add_executable(imu_node src/imu_node.cpp)
rosbuild_add_executable(control_node src/control_node.cpp)
target_link_libraries(imu_node libz800)
