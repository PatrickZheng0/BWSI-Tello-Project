# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ericvo/BWSI-Tello-Project/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ericvo/BWSI-Tello-Project/catkin_ws/build

# Utility rule file for just_drone_gencpp.

# Include the progress variables for this target.
include just_drone/CMakeFiles/just_drone_gencpp.dir/progress.make

just_drone_gencpp: just_drone/CMakeFiles/just_drone_gencpp.dir/build.make

.PHONY : just_drone_gencpp

# Rule to build all files generated by this target.
just_drone/CMakeFiles/just_drone_gencpp.dir/build: just_drone_gencpp

.PHONY : just_drone/CMakeFiles/just_drone_gencpp.dir/build

just_drone/CMakeFiles/just_drone_gencpp.dir/clean:
	cd /home/ericvo/BWSI-Tello-Project/catkin_ws/build/just_drone && $(CMAKE_COMMAND) -P CMakeFiles/just_drone_gencpp.dir/cmake_clean.cmake
.PHONY : just_drone/CMakeFiles/just_drone_gencpp.dir/clean

just_drone/CMakeFiles/just_drone_gencpp.dir/depend:
	cd /home/ericvo/BWSI-Tello-Project/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ericvo/BWSI-Tello-Project/catkin_ws/src /home/ericvo/BWSI-Tello-Project/catkin_ws/src/just_drone /home/ericvo/BWSI-Tello-Project/catkin_ws/build /home/ericvo/BWSI-Tello-Project/catkin_ws/build/just_drone /home/ericvo/BWSI-Tello-Project/catkin_ws/build/just_drone/CMakeFiles/just_drone_gencpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : just_drone/CMakeFiles/just_drone_gencpp.dir/depend

