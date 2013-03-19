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

include_directories(${PROJECT_SOURCE_DIR}/src/)

#Libs
rosbuild_add_library(utilities_LIBS src/UtilitiesBlock.cpp)				# Utils
rosbuild_add_library(ransac_LIBS src/ransac/RSTEstimator.cpp)			# RANSAC


rosbuild_add_library(init_block_LIBS src/ICPInitializationBlock.cpp)	# INIT Block
target_link_libraries(init_block_LIBS ransac_LIBS)


rosbuild_add_library(icp_LIBS src/ICPBlock.cpp)							# ICP p2p

rosbuild_add_library(icp_p2pl_LIBS src/ICP_point2plane.cpp)				# ICP p2pl
target_link_libraries(icp_p2pl_LIBS icp_LIBS)

rosbuild_add_library(icp_pl2pl_LIBS src/ICP_plane2plane.cpp)			# ICP pl2pl
target_link_libraries(icp_pl2pl_LIBS icp_LIBS)

#rosbuild_add_library(surfels_lib src/surfels_rgbd.cpp)					#Surfels


#rosbuild_add_library(doc_lib src/MakeDoc.cpp)							#Doc

#rosbuild_add_library(dataset_LIB src/DatasetAnalyzer.cpp)				#dataset(artoolkit)
#target_link_libraries(dataset_LIB  utilities_LIBS)

#Exe
#rosbuild_add_executable(world_node src/worldmapBlock.cpp)				# World NODE
#target_link_libraries(world_node utilities_LIBS)

rosbuild_add_executable(world_node2 src/worldmapBlock2.cpp)				# World2 NODE
target_link_libraries(world_node2 utilities_LIBS
								  init_block_LIBS)

#Demos
#rosbuild_add_executable(test_icp_block src/test_files/test_ICP.cpp)
#target_link_libraries(test_icp_block utilities_LIBS
#									 init_block_LIBS
#									 icp_LIBS
#									 icp_pl2pl_LIBS
#									 dataset_LIB
#									 doc_lib)


#rosbuild_add_executable(demo_icp src/demos/demo_icp.cpp)					# DEMO icp p2p
#target_link_libraries(demo_icp icp_LIBS 
#							   icp_pl2pl_LIBS
#						       init_block_LIBS
#						       utilities_LIBS)

#rosbuild_add_executable(demo_icp_read_dataset src/demos/demo_icp_read_dataset.cpp)					# DEMO icp p2p
#target_link_libraries(demo_icp_read_dataset icp_LIBS 
#											icp_pl2pl_LIBS
#											init_block_LIBS
#											utilities_LIBS)

#rosbuild_add_executable(demo_icp_keyframes src/demos/demo_icp_keyframes.cpp)					# DEMO icp p2p with key frames
#target_link_libraries(demo_icp_keyframes icp_LIBS 
#										 init_block_LIBS
#										 utilities_LIBS)

#rosbuild_add_executable(demo_align_publisher src/demos/demo_align_publisher.cpp)				# DEMO icp p2p with folder
#target_link_libraries(demo_align_publisher icp_LIBS 
#										   init_block_LIBS)

rosbuild_add_executable(demo_with_odometry src/demos/demo_publisher_odometry.cpp)				# DEMO icp p2p with folder and odometry
target_link_libraries(demo_with_odometry   init_block_LIBS
										   icp_LIBS
										   icp_p2pl_LIBS
										   icp_pl2pl_LIBS)


#rosbuild_add_executable(test_filtro src/test_files/test_filtro.cpp)		
#target_link_libraries(test_filtro utilities_LIBS)

#rosbuild_add_executable(dataset src/DatasetAnalyzer.cpp)
#target_link_libraries(dataset utilities_LIBS)


#rosbuild_add_executable(coco src/coco.cpp)				# DEMO icp p2p with folder and odometry
#target_link_libraries(coco utilities_LIBS)
						
