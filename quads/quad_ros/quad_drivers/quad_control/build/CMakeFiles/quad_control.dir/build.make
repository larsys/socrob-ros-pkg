# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/quadbase/ros/quad_ros/quad_drivers/quad_control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/quadbase/ros/quad_ros/quad_drivers/quad_control/build

# Include any dependencies generated for this target.
include CMakeFiles/quad_control.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quad_control.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quad_control.dir/flags.make

CMakeFiles/quad_control.dir/src/quad_control.o: CMakeFiles/quad_control.dir/flags.make
CMakeFiles/quad_control.dir/src/quad_control.o: ../src/quad_control.cpp
CMakeFiles/quad_control.dir/src/quad_control.o: ../manifest.xml
CMakeFiles/quad_control.dir/src/quad_control.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/quad_control.dir/src/quad_control.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/quad_control.dir/src/quad_control.o: /opt/ros/fuerte/share/roscpp/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/quadbase/ros/quad_ros/quad_drivers/quad_control/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quad_control.dir/src/quad_control.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/quad_control.dir/src/quad_control.o -c /home/quadbase/ros/quad_ros/quad_drivers/quad_control/src/quad_control.cpp

CMakeFiles/quad_control.dir/src/quad_control.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quad_control.dir/src/quad_control.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/quadbase/ros/quad_ros/quad_drivers/quad_control/src/quad_control.cpp > CMakeFiles/quad_control.dir/src/quad_control.i

CMakeFiles/quad_control.dir/src/quad_control.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quad_control.dir/src/quad_control.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/quadbase/ros/quad_ros/quad_drivers/quad_control/src/quad_control.cpp -o CMakeFiles/quad_control.dir/src/quad_control.s

CMakeFiles/quad_control.dir/src/quad_control.o.requires:
.PHONY : CMakeFiles/quad_control.dir/src/quad_control.o.requires

CMakeFiles/quad_control.dir/src/quad_control.o.provides: CMakeFiles/quad_control.dir/src/quad_control.o.requires
	$(MAKE) -f CMakeFiles/quad_control.dir/build.make CMakeFiles/quad_control.dir/src/quad_control.o.provides.build
.PHONY : CMakeFiles/quad_control.dir/src/quad_control.o.provides

CMakeFiles/quad_control.dir/src/quad_control.o.provides.build: CMakeFiles/quad_control.dir/src/quad_control.o

# Object files for target quad_control
quad_control_OBJECTS = \
"CMakeFiles/quad_control.dir/src/quad_control.o"

# External object files for target quad_control
quad_control_EXTERNAL_OBJECTS =

../bin/quad_control: CMakeFiles/quad_control.dir/src/quad_control.o
../bin/quad_control: CMakeFiles/quad_control.dir/build.make
../bin/quad_control: CMakeFiles/quad_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/quad_control"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quad_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quad_control.dir/build: ../bin/quad_control
.PHONY : CMakeFiles/quad_control.dir/build

CMakeFiles/quad_control.dir/requires: CMakeFiles/quad_control.dir/src/quad_control.o.requires
.PHONY : CMakeFiles/quad_control.dir/requires

CMakeFiles/quad_control.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quad_control.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quad_control.dir/clean

CMakeFiles/quad_control.dir/depend:
	cd /home/quadbase/ros/quad_ros/quad_drivers/quad_control/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/quadbase/ros/quad_ros/quad_drivers/quad_control /home/quadbase/ros/quad_ros/quad_drivers/quad_control /home/quadbase/ros/quad_ros/quad_drivers/quad_control/build /home/quadbase/ros/quad_ros/quad_drivers/quad_control/build /home/quadbase/ros/quad_ros/quad_drivers/quad_control/build/CMakeFiles/quad_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quad_control.dir/depend

