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
CMAKE_SOURCE_DIR = /home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense

# Utility rule file for pioneer_dual_realsense_autogen.

# Include the progress variables for this target.
include src/CMakeFiles/pioneer_dual_realsense_autogen.dir/progress.make

src/CMakeFiles/pioneer_dual_realsense_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target pioneer_dual_realsense"
	cd /home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense/src && /usr/bin/cmake -E cmake_autogen /home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense/src/CMakeFiles/pioneer_dual_realsense_autogen.dir/AutogenInfo.json ""

pioneer_dual_realsense_autogen: src/CMakeFiles/pioneer_dual_realsense_autogen
pioneer_dual_realsense_autogen: src/CMakeFiles/pioneer_dual_realsense_autogen.dir/build.make

.PHONY : pioneer_dual_realsense_autogen

# Rule to build all files generated by this target.
src/CMakeFiles/pioneer_dual_realsense_autogen.dir/build: pioneer_dual_realsense_autogen

.PHONY : src/CMakeFiles/pioneer_dual_realsense_autogen.dir/build

src/CMakeFiles/pioneer_dual_realsense_autogen.dir/clean:
	cd /home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense/src && $(CMAKE_COMMAND) -P CMakeFiles/pioneer_dual_realsense_autogen.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/pioneer_dual_realsense_autogen.dir/clean

src/CMakeFiles/pioneer_dual_realsense_autogen.dir/depend:
	cd /home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense /home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense/src /home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense /home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense/src /home/pioneernuc/robocomp/components/robocomp-pioneer/components/pioneer_dual_realsense/src/CMakeFiles/pioneer_dual_realsense_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/pioneer_dual_realsense_autogen.dir/depend

