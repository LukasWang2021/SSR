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
include joint_constraint/CMakeFiles/joint_constraint.dir/depend.make

# Include the progress variables for this target.
include joint_constraint/CMakeFiles/joint_constraint.dir/progress.make

# Include the compile flags for this target's objects.
include joint_constraint/CMakeFiles/joint_constraint.dir/flags.make

joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o: joint_constraint/CMakeFiles/joint_constraint.dir/flags.make
joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o: /home/ld/git_work/mc_application/Motion_controller/src/joint_constraint/src/joint_constraint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ld/git_work/mc_application/Motion_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o"
	cd /home/ld/git_work/mc_application/Motion_controller/build/joint_constraint && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o -c /home/ld/git_work/mc_application/Motion_controller/src/joint_constraint/src/joint_constraint.cpp

joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.i"
	cd /home/ld/git_work/mc_application/Motion_controller/build/joint_constraint && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ld/git_work/mc_application/Motion_controller/src/joint_constraint/src/joint_constraint.cpp > CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.i

joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.s"
	cd /home/ld/git_work/mc_application/Motion_controller/build/joint_constraint && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ld/git_work/mc_application/Motion_controller/src/joint_constraint/src/joint_constraint.cpp -o CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.s

joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o.requires:

.PHONY : joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o.requires

joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o.provides: joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o.requires
	$(MAKE) -f joint_constraint/CMakeFiles/joint_constraint.dir/build.make joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o.provides.build
.PHONY : joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o.provides

joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o.provides.build: joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o


# Object files for target joint_constraint
joint_constraint_OBJECTS = \
"CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o"

# External object files for target joint_constraint
joint_constraint_EXTERNAL_OBJECTS =

/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: joint_constraint/CMakeFiles/joint_constraint.dir/build.make
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libstdc++.so.6
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libpthread.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/librt.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libz.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/liblzma.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libxml2.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libyaml-cpp.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/local/lib/libnanomsg.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicuuc.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicudata.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libdl.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /home/ld/git_work/mc_application/Motion_controller/install/lib/libbasic_alg.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libstdc++.so.6
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libpthread.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/librt.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libz.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/liblzma.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libxml2.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libyaml-cpp.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/local/lib/libnanomsg.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicuuc.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicudata.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libdl.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/local/lib/libf2c.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/local/lib/libblas.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: /usr/local/crosstool/zcu102/usr/local/lib/liblapack.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so: joint_constraint/CMakeFiles/joint_constraint.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ld/git_work/mc_application/Motion_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so"
	cd /home/ld/git_work/mc_application/Motion_controller/build/joint_constraint && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joint_constraint.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
joint_constraint/CMakeFiles/joint_constraint.dir/build: /home/ld/git_work/mc_application/Motion_controller/install/lib/libjoint_constraint.so

.PHONY : joint_constraint/CMakeFiles/joint_constraint.dir/build

joint_constraint/CMakeFiles/joint_constraint.dir/requires: joint_constraint/CMakeFiles/joint_constraint.dir/src/joint_constraint.cpp.o.requires

.PHONY : joint_constraint/CMakeFiles/joint_constraint.dir/requires

joint_constraint/CMakeFiles/joint_constraint.dir/clean:
	cd /home/ld/git_work/mc_application/Motion_controller/build/joint_constraint && $(CMAKE_COMMAND) -P CMakeFiles/joint_constraint.dir/cmake_clean.cmake
.PHONY : joint_constraint/CMakeFiles/joint_constraint.dir/clean

joint_constraint/CMakeFiles/joint_constraint.dir/depend:
	cd /home/ld/git_work/mc_application/Motion_controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ld/git_work/mc_application/Motion_controller/src /home/ld/git_work/mc_application/Motion_controller/src/joint_constraint /home/ld/git_work/mc_application/Motion_controller/build /home/ld/git_work/mc_application/Motion_controller/build/joint_constraint /home/ld/git_work/mc_application/Motion_controller/build/joint_constraint/CMakeFiles/joint_constraint.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : joint_constraint/CMakeFiles/joint_constraint.dir/depend

