# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/ld/git_work/mc_application/Motion_controller/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ld/git_work/mc_application/Motion_controller/build

# Include any dependencies generated for this target.
include test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/depend.make

# Include the progress variables for this target.
include test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/progress.make

# Include the compile flags for this target's objects.
include test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/flags.make

test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o: test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/flags.make
test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o: /home/ld/git_work/mc_application/Motion_controller/src/test/tp_comm_test/src/test_motion_control/online_trajectory_ratio_get.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ld/git_work/mc_application/Motion_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o"
	cd /home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_motion_control && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o -c /home/ld/git_work/mc_application/Motion_controller/src/test/tp_comm_test/src/test_motion_control/online_trajectory_ratio_get.cpp

test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.i"
	cd /home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_motion_control && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ld/git_work/mc_application/Motion_controller/src/test/tp_comm_test/src/test_motion_control/online_trajectory_ratio_get.cpp > CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.i

test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.s"
	cd /home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_motion_control && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ld/git_work/mc_application/Motion_controller/src/test/tp_comm_test/src/test_motion_control/online_trajectory_ratio_get.cpp -o CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.s

test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o.requires:

.PHONY : test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o.requires

test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o.provides: test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o.requires
	$(MAKE) -f test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/build.make test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o.provides.build
.PHONY : test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o.provides

test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o.provides.build: test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o


# Object files for target online_trajectory_ratio_get
online_trajectory_ratio_get_OBJECTS = \
"CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o"

# External object files for target online_trajectory_ratio_get
online_trajectory_ratio_get_EXTERNAL_OBJECTS =

/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/build.make
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libstdc++.so.6
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libpthread.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/librt.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libz.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/liblzma.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libxml2.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libyaml-cpp.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/local/lib/libnanomsg.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicuuc.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicudata.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libdl.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /home/ld/git_work/mc_application/Motion_controller/install/lib/libtp_comm_test.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /home/ld/git_work/mc_application/Motion_controller/install/lib/libprotos.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /home/ld/git_work/mc_application/Motion_controller/install/lib/libpb.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /home/ld/git_work/mc_application/Motion_controller/install/lib/libthread_help.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libstdc++.so.6
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libpthread.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/librt.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libz.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/liblzma.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libxml2.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libyaml-cpp.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/local/lib/libnanomsg.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicuuc.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicudata.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libdl.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get: test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ld/git_work/mc_application/Motion_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get"
	cd /home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_motion_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/online_trajectory_ratio_get.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/build: /home/ld/git_work/mc_application/Motion_controller/install/bin/tp_comm_test/motion_control/online_trajectory_ratio_get

.PHONY : test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/build

test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/requires: test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/online_trajectory_ratio_get.cpp.o.requires

.PHONY : test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/requires

test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/clean:
	cd /home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_motion_control && $(CMAKE_COMMAND) -P CMakeFiles/online_trajectory_ratio_get.dir/cmake_clean.cmake
.PHONY : test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/clean

test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/depend:
	cd /home/ld/git_work/mc_application/Motion_controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ld/git_work/mc_application/Motion_controller/src /home/ld/git_work/mc_application/Motion_controller/src/test/tp_comm_test/src/test_motion_control /home/ld/git_work/mc_application/Motion_controller/build /home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_motion_control /home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/tp_comm_test/src/test_motion_control/CMakeFiles/online_trajectory_ratio_get.dir/depend

