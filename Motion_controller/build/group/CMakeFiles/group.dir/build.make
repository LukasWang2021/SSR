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
include group/CMakeFiles/group.dir/depend.make

# Include the progress variables for this target.
include group/CMakeFiles/group.dir/progress.make

# Include the compile flags for this target's objects.
include group/CMakeFiles/group.dir/flags.make

group/CMakeFiles/group.dir/src/group.cpp.o: group/CMakeFiles/group.dir/flags.make
group/CMakeFiles/group.dir/src/group.cpp.o: /home/ld/git_work/mc_application/Motion_controller/src/group/src/group.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ld/git_work/mc_application/Motion_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object group/CMakeFiles/group.dir/src/group.cpp.o"
	cd /home/ld/git_work/mc_application/Motion_controller/build/group && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/group.dir/src/group.cpp.o -c /home/ld/git_work/mc_application/Motion_controller/src/group/src/group.cpp

group/CMakeFiles/group.dir/src/group.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/group.dir/src/group.cpp.i"
	cd /home/ld/git_work/mc_application/Motion_controller/build/group && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ld/git_work/mc_application/Motion_controller/src/group/src/group.cpp > CMakeFiles/group.dir/src/group.cpp.i

group/CMakeFiles/group.dir/src/group.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/group.dir/src/group.cpp.s"
	cd /home/ld/git_work/mc_application/Motion_controller/build/group && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ld/git_work/mc_application/Motion_controller/src/group/src/group.cpp -o CMakeFiles/group.dir/src/group.cpp.s

group/CMakeFiles/group.dir/src/group.cpp.o.requires:

.PHONY : group/CMakeFiles/group.dir/src/group.cpp.o.requires

group/CMakeFiles/group.dir/src/group.cpp.o.provides: group/CMakeFiles/group.dir/src/group.cpp.o.requires
	$(MAKE) -f group/CMakeFiles/group.dir/build.make group/CMakeFiles/group.dir/src/group.cpp.o.provides.build
.PHONY : group/CMakeFiles/group.dir/src/group.cpp.o.provides

group/CMakeFiles/group.dir/src/group.cpp.o.provides.build: group/CMakeFiles/group.dir/src/group.cpp.o


group/CMakeFiles/group.dir/src/group_fdb.cpp.o: group/CMakeFiles/group.dir/flags.make
group/CMakeFiles/group.dir/src/group_fdb.cpp.o: /home/ld/git_work/mc_application/Motion_controller/src/group/src/group_fdb.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ld/git_work/mc_application/Motion_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object group/CMakeFiles/group.dir/src/group_fdb.cpp.o"
	cd /home/ld/git_work/mc_application/Motion_controller/build/group && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/group.dir/src/group_fdb.cpp.o -c /home/ld/git_work/mc_application/Motion_controller/src/group/src/group_fdb.cpp

group/CMakeFiles/group.dir/src/group_fdb.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/group.dir/src/group_fdb.cpp.i"
	cd /home/ld/git_work/mc_application/Motion_controller/build/group && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ld/git_work/mc_application/Motion_controller/src/group/src/group_fdb.cpp > CMakeFiles/group.dir/src/group_fdb.cpp.i

group/CMakeFiles/group.dir/src/group_fdb.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/group.dir/src/group_fdb.cpp.s"
	cd /home/ld/git_work/mc_application/Motion_controller/build/group && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ld/git_work/mc_application/Motion_controller/src/group/src/group_fdb.cpp -o CMakeFiles/group.dir/src/group_fdb.cpp.s

group/CMakeFiles/group.dir/src/group_fdb.cpp.o.requires:

.PHONY : group/CMakeFiles/group.dir/src/group_fdb.cpp.o.requires

group/CMakeFiles/group.dir/src/group_fdb.cpp.o.provides: group/CMakeFiles/group.dir/src/group_fdb.cpp.o.requires
	$(MAKE) -f group/CMakeFiles/group.dir/build.make group/CMakeFiles/group.dir/src/group_fdb.cpp.o.provides.build
.PHONY : group/CMakeFiles/group.dir/src/group_fdb.cpp.o.provides

group/CMakeFiles/group.dir/src/group_fdb.cpp.o.provides.build: group/CMakeFiles/group.dir/src/group_fdb.cpp.o


group/CMakeFiles/group.dir/src/group_sm.cpp.o: group/CMakeFiles/group.dir/flags.make
group/CMakeFiles/group.dir/src/group_sm.cpp.o: /home/ld/git_work/mc_application/Motion_controller/src/group/src/group_sm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ld/git_work/mc_application/Motion_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object group/CMakeFiles/group.dir/src/group_sm.cpp.o"
	cd /home/ld/git_work/mc_application/Motion_controller/build/group && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/group.dir/src/group_sm.cpp.o -c /home/ld/git_work/mc_application/Motion_controller/src/group/src/group_sm.cpp

group/CMakeFiles/group.dir/src/group_sm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/group.dir/src/group_sm.cpp.i"
	cd /home/ld/git_work/mc_application/Motion_controller/build/group && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ld/git_work/mc_application/Motion_controller/src/group/src/group_sm.cpp > CMakeFiles/group.dir/src/group_sm.cpp.i

group/CMakeFiles/group.dir/src/group_sm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/group.dir/src/group_sm.cpp.s"
	cd /home/ld/git_work/mc_application/Motion_controller/build/group && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ld/git_work/mc_application/Motion_controller/src/group/src/group_sm.cpp -o CMakeFiles/group.dir/src/group_sm.cpp.s

group/CMakeFiles/group.dir/src/group_sm.cpp.o.requires:

.PHONY : group/CMakeFiles/group.dir/src/group_sm.cpp.o.requires

group/CMakeFiles/group.dir/src/group_sm.cpp.o.provides: group/CMakeFiles/group.dir/src/group_sm.cpp.o.requires
	$(MAKE) -f group/CMakeFiles/group.dir/build.make group/CMakeFiles/group.dir/src/group_sm.cpp.o.provides.build
.PHONY : group/CMakeFiles/group.dir/src/group_sm.cpp.o.provides

group/CMakeFiles/group.dir/src/group_sm.cpp.o.provides.build: group/CMakeFiles/group.dir/src/group_sm.cpp.o


# Object files for target group
group_OBJECTS = \
"CMakeFiles/group.dir/src/group.cpp.o" \
"CMakeFiles/group.dir/src/group_fdb.cpp.o" \
"CMakeFiles/group.dir/src/group_sm.cpp.o"

# External object files for target group
group_EXTERNAL_OBJECTS =

/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: group/CMakeFiles/group.dir/src/group.cpp.o
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: group/CMakeFiles/group.dir/src/group_fdb.cpp.o
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: group/CMakeFiles/group.dir/src/group_sm.cpp.o
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: group/CMakeFiles/group.dir/build.make
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libstdc++.so.6
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libpthread.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/librt.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libz.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/liblzma.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libxml2.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libyaml-cpp.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/local/lib/libnanomsg.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicuuc.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicudata.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libdl.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /home/ld/git_work/mc_application/Motion_controller/install/lib/libaxis.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /home/ld/git_work/mc_application/Motion_controller/install/lib/liberror_queue.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /home/ld/git_work/mc_application/Motion_controller/install/lib/libservo_comm.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /home/ld/git_work/mc_application/Motion_controller/install/lib/libcore_comm_system.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /home/ld/git_work/mc_application/Motion_controller/install/lib/libsystem_model_manager.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /home/ld/git_work/mc_application/Motion_controller/install/lib/liblog_manager_producer.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /home/ld/git_work/mc_application/Motion_controller/install/lib/libyaml_help.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /home/ld/git_work/mc_application/Motion_controller/install/lib/libthread_help.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /home/ld/git_work/mc_application/Motion_controller/install/lib/libxml_help.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /home/ld/git_work/mc_application/Motion_controller/install/lib/libalgorithm_base.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libstdc++.so.6
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libpthread.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/librt.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libz.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/liblzma.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libxml2.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libyaml-cpp.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/local/lib/libnanomsg.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicuuc.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicudata.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libdl.so
/home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so: group/CMakeFiles/group.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ld/git_work/mc_application/Motion_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library /home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so"
	cd /home/ld/git_work/mc_application/Motion_controller/build/group && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/group.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
group/CMakeFiles/group.dir/build: /home/ld/git_work/mc_application/Motion_controller/install/lib/libgroup.so

.PHONY : group/CMakeFiles/group.dir/build

group/CMakeFiles/group.dir/requires: group/CMakeFiles/group.dir/src/group.cpp.o.requires
group/CMakeFiles/group.dir/requires: group/CMakeFiles/group.dir/src/group_fdb.cpp.o.requires
group/CMakeFiles/group.dir/requires: group/CMakeFiles/group.dir/src/group_sm.cpp.o.requires

.PHONY : group/CMakeFiles/group.dir/requires

group/CMakeFiles/group.dir/clean:
	cd /home/ld/git_work/mc_application/Motion_controller/build/group && $(CMAKE_COMMAND) -P CMakeFiles/group.dir/cmake_clean.cmake
.PHONY : group/CMakeFiles/group.dir/clean

group/CMakeFiles/group.dir/depend:
	cd /home/ld/git_work/mc_application/Motion_controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ld/git_work/mc_application/Motion_controller/src /home/ld/git_work/mc_application/Motion_controller/src/group /home/ld/git_work/mc_application/Motion_controller/build /home/ld/git_work/mc_application/Motion_controller/build/group /home/ld/git_work/mc_application/Motion_controller/build/group/CMakeFiles/group.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : group/CMakeFiles/group.dir/depend

