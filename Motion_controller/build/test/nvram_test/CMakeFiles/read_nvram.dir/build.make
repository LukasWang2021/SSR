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
include test/nvram_test/CMakeFiles/read_nvram.dir/depend.make

# Include the progress variables for this target.
include test/nvram_test/CMakeFiles/read_nvram.dir/progress.make

# Include the compile flags for this target's objects.
include test/nvram_test/CMakeFiles/read_nvram.dir/flags.make

test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o: test/nvram_test/CMakeFiles/read_nvram.dir/flags.make
test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o: /home/ld/git_work/mc_application/Motion_controller/src/test/nvram_test/read_nvram.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ld/git_work/mc_application/Motion_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o"
	cd /home/ld/git_work/mc_application/Motion_controller/build/test/nvram_test && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/read_nvram.dir/read_nvram.cpp.o -c /home/ld/git_work/mc_application/Motion_controller/src/test/nvram_test/read_nvram.cpp

test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/read_nvram.dir/read_nvram.cpp.i"
	cd /home/ld/git_work/mc_application/Motion_controller/build/test/nvram_test && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ld/git_work/mc_application/Motion_controller/src/test/nvram_test/read_nvram.cpp > CMakeFiles/read_nvram.dir/read_nvram.cpp.i

test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/read_nvram.dir/read_nvram.cpp.s"
	cd /home/ld/git_work/mc_application/Motion_controller/build/test/nvram_test && /usr/local/aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ld/git_work/mc_application/Motion_controller/src/test/nvram_test/read_nvram.cpp -o CMakeFiles/read_nvram.dir/read_nvram.cpp.s

test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o.requires:

.PHONY : test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o.requires

test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o.provides: test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o.requires
	$(MAKE) -f test/nvram_test/CMakeFiles/read_nvram.dir/build.make test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o.provides.build
.PHONY : test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o.provides

test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o.provides.build: test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o


# Object files for target read_nvram
read_nvram_OBJECTS = \
"CMakeFiles/read_nvram.dir/read_nvram.cpp.o"

# External object files for target read_nvram
read_nvram_EXTERNAL_OBJECTS =

/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: test/nvram_test/CMakeFiles/read_nvram.dir/build.make
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libstdc++.so.6
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libpthread.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/librt.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libz.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/liblzma.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libxml2.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libyaml-cpp.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/local/lib/libnanomsg.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicuuc.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicudata.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libdl.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /home/ld/git_work/mc_application/Motion_controller/install/lib/libnvram_handler.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /home/ld/git_work/mc_application/Motion_controller/install/lib/librtm_spi.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libstdc++.so.6
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libpthread.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/librt.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libz.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/liblzma.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libxml2.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libyaml-cpp.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/local/lib/libnanomsg.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicuuc.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libicudata.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: /usr/local/crosstool/zcu102/usr/lib/aarch64-linux-gnu/libdl.so
/home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram: test/nvram_test/CMakeFiles/read_nvram.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ld/git_work/mc_application/Motion_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram"
	cd /home/ld/git_work/mc_application/Motion_controller/build/test/nvram_test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/read_nvram.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/nvram_test/CMakeFiles/read_nvram.dir/build: /home/ld/git_work/mc_application/Motion_controller/install/bin/nvram_test/read_nvram

.PHONY : test/nvram_test/CMakeFiles/read_nvram.dir/build

test/nvram_test/CMakeFiles/read_nvram.dir/requires: test/nvram_test/CMakeFiles/read_nvram.dir/read_nvram.cpp.o.requires

.PHONY : test/nvram_test/CMakeFiles/read_nvram.dir/requires

test/nvram_test/CMakeFiles/read_nvram.dir/clean:
	cd /home/ld/git_work/mc_application/Motion_controller/build/test/nvram_test && $(CMAKE_COMMAND) -P CMakeFiles/read_nvram.dir/cmake_clean.cmake
.PHONY : test/nvram_test/CMakeFiles/read_nvram.dir/clean

test/nvram_test/CMakeFiles/read_nvram.dir/depend:
	cd /home/ld/git_work/mc_application/Motion_controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ld/git_work/mc_application/Motion_controller/src /home/ld/git_work/mc_application/Motion_controller/src/test/nvram_test /home/ld/git_work/mc_application/Motion_controller/build /home/ld/git_work/mc_application/Motion_controller/build/test/nvram_test /home/ld/git_work/mc_application/Motion_controller/build/test/nvram_test/CMakeFiles/read_nvram.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/nvram_test/CMakeFiles/read_nvram.dir/depend

