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
CMAKE_SOURCE_DIR = /home/quadbase/ros/quad_ros/quad_drivers/quad_link

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/quadbase/ros/quad_ros/quad_drivers/quad_link/build

# Include any dependencies generated for this target.
include CMakeFiles/quad_link.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quad_link.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quad_link.dir/flags.make

CMakeFiles/quad_link.dir/src/quad_link.o: CMakeFiles/quad_link.dir/flags.make
CMakeFiles/quad_link.dir/src/quad_link.o: ../src/quad_link.cpp
CMakeFiles/quad_link.dir/src/quad_link.o: ../manifest.xml
CMakeFiles/quad_link.dir/src/quad_link.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/quad_link.dir/src/quad_link.o: /opt/ros/fuerte/share/roscpp/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/quadbase/ros/quad_ros/quad_drivers/quad_link/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/quad_link.dir/src/quad_link.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/quad_link.dir/src/quad_link.o -c /home/quadbase/ros/quad_ros/quad_drivers/quad_link/src/quad_link.cpp

CMakeFiles/quad_link.dir/src/quad_link.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quad_link.dir/src/quad_link.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/quadbase/ros/quad_ros/quad_drivers/quad_link/src/quad_link.cpp > CMakeFiles/quad_link.dir/src/quad_link.i

CMakeFiles/quad_link.dir/src/quad_link.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quad_link.dir/src/quad_link.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/quadbase/ros/quad_ros/quad_drivers/quad_link/src/quad_link.cpp -o CMakeFiles/quad_link.dir/src/quad_link.s

CMakeFiles/quad_link.dir/src/quad_link.o.requires:
.PHONY : CMakeFiles/quad_link.dir/src/quad_link.o.requires

CMakeFiles/quad_link.dir/src/quad_link.o.provides: CMakeFiles/quad_link.dir/src/quad_link.o.requires
	$(MAKE) -f CMakeFiles/quad_link.dir/build.make CMakeFiles/quad_link.dir/src/quad_link.o.provides.build
.PHONY : CMakeFiles/quad_link.dir/src/quad_link.o.provides

CMakeFiles/quad_link.dir/src/quad_link.o.provides.build: CMakeFiles/quad_link.dir/src/quad_link.o

# Object files for target quad_link
quad_link_OBJECTS = \
"CMakeFiles/quad_link.dir/src/quad_link.o"

# External object files for target quad_link
quad_link_EXTERNAL_OBJECTS =

../bin/quad_link: CMakeFiles/quad_link.dir/src/quad_link.o
../bin/quad_link: CMakeFiles/quad_link.dir/build.make
../bin/quad_link: CMakeFiles/quad_link.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/quad_link"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quad_link.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quad_link.dir/build: ../bin/quad_link
.PHONY : CMakeFiles/quad_link.dir/build

CMakeFiles/quad_link.dir/requires: CMakeFiles/quad_link.dir/src/quad_link.o.requires
.PHONY : CMakeFiles/quad_link.dir/requires

CMakeFiles/quad_link.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quad_link.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quad_link.dir/clean

CMakeFiles/quad_link.dir/depend:
	cd /home/quadbase/ros/quad_ros/quad_drivers/quad_link/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/quadbase/ros/quad_ros/quad_drivers/quad_link /home/quadbase/ros/quad_ros/quad_drivers/quad_link /home/quadbase/ros/quad_ros/quad_drivers/quad_link/build /home/quadbase/ros/quad_ros/quad_drivers/quad_link/build /home/quadbase/ros/quad_ros/quad_drivers/quad_link/build/CMakeFiles/quad_link.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quad_link.dir/depend
