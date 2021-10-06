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

# Utility rule file for ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src.

# Include the progress variables for this target.
include src/CMakeFiles/ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src.dir/progress.make

ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src: src/CMakeFiles/ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src.dir/build.make
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/pioneernuc/robocomp/interfaces/IDSLs/CommonBehavior.idsl /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src/CommonBehavior.ice"
	cd /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src && robocompdsl /home/pioneernuc/robocomp/interfaces/IDSLs/CommonBehavior.idsl /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src/CommonBehavior.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/pioneernuc/robocomp/interfaces/IDSLs/CameraSimple.idsl /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src/CameraSimple.ice"
	cd /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src && robocompdsl /home/pioneernuc/robocomp/interfaces/IDSLs/CameraSimple.idsl /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src/CameraSimple.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/pioneernuc/robocomp/interfaces/IDSLs/MonitorBase.idsl /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src/MonitorBase.ice"
	cd /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src && robocompdsl /home/pioneernuc/robocomp/interfaces/IDSLs/MonitorBase.idsl /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src/MonitorBase.ice
.PHONY : ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src

# Rule to build all files generated by this target.
src/CMakeFiles/ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src.dir/build: ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src

.PHONY : src/CMakeFiles/ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src.dir/build

src/CMakeFiles/ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src.dir/clean:
	cd /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src && $(CMAKE_COMMAND) -P CMakeFiles/ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src.dir/clean

src/CMakeFiles/ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src.dir/depend:
	cd /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src /home/pioneernuc/robocomp/components/robocomp-pioneer/components/usb_camera/src/CMakeFiles/ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/ICES__home_pioneernuc_robocomp_components_robocomp-pioneer_components_usb_camera_src.dir/depend

