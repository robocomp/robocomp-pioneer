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
CMAKE_SOURCE_DIR = /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera

# Utility rule file for usb_camera_autogen.

# Include the progress variables for this target.
include src/CMakeFiles/usb_camera_autogen.dir/progress.make

src/CMakeFiles/usb_camera_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target usb_camera"
	cd /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src && /usr/bin/cmake -E cmake_autogen /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src/CMakeFiles/usb_camera_autogen.dir/AutogenInfo.json ""

usb_camera_autogen: src/CMakeFiles/usb_camera_autogen
usb_camera_autogen: src/CMakeFiles/usb_camera_autogen.dir/build.make

.PHONY : usb_camera_autogen

# Rule to build all files generated by this target.
src/CMakeFiles/usb_camera_autogen.dir/build: usb_camera_autogen

.PHONY : src/CMakeFiles/usb_camera_autogen.dir/build

src/CMakeFiles/usb_camera_autogen.dir/clean:
	cd /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src && $(CMAKE_COMMAND) -P CMakeFiles/usb_camera_autogen.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/usb_camera_autogen.dir/clean

src/CMakeFiles/usb_camera_autogen.dir/depend:
	cd /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src/CMakeFiles/usb_camera_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/usb_camera_autogen.dir/depend

