# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/nakkeslengprosjekt/asdf

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nakkeslengprosjekt/asdf/build

# Include any dependencies generated for this target.
include CMakeFiles/examples_common.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/examples_common.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/examples_common.dir/flags.make

CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o: CMakeFiles/examples_common.dir/flags.make
CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o: ../src/source/examples_common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nakkeslengprosjekt/asdf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o -c /home/nakkeslengprosjekt/asdf/src/source/examples_common.cpp

CMakeFiles/examples_common.dir/src/source/examples_common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/examples_common.dir/src/source/examples_common.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nakkeslengprosjekt/asdf/src/source/examples_common.cpp > CMakeFiles/examples_common.dir/src/source/examples_common.cpp.i

CMakeFiles/examples_common.dir/src/source/examples_common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/examples_common.dir/src/source/examples_common.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nakkeslengprosjekt/asdf/src/source/examples_common.cpp -o CMakeFiles/examples_common.dir/src/source/examples_common.cpp.s

CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o.requires:

.PHONY : CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o.requires

CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o.provides: CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o.requires
	$(MAKE) -f CMakeFiles/examples_common.dir/build.make CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o.provides.build
.PHONY : CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o.provides

CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o.provides.build: CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o


# Object files for target examples_common
examples_common_OBJECTS = \
"CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o"

# External object files for target examples_common
examples_common_EXTERNAL_OBJECTS =

libexamples_common.a: CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o
libexamples_common.a: CMakeFiles/examples_common.dir/build.make
libexamples_common.a: CMakeFiles/examples_common.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nakkeslengprosjekt/asdf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libexamples_common.a"
	$(CMAKE_COMMAND) -P CMakeFiles/examples_common.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/examples_common.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/examples_common.dir/build: libexamples_common.a

.PHONY : CMakeFiles/examples_common.dir/build

CMakeFiles/examples_common.dir/requires: CMakeFiles/examples_common.dir/src/source/examples_common.cpp.o.requires

.PHONY : CMakeFiles/examples_common.dir/requires

CMakeFiles/examples_common.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/examples_common.dir/cmake_clean.cmake
.PHONY : CMakeFiles/examples_common.dir/clean

CMakeFiles/examples_common.dir/depend:
	cd /home/nakkeslengprosjekt/asdf/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nakkeslengprosjekt/asdf /home/nakkeslengprosjekt/asdf /home/nakkeslengprosjekt/asdf/build /home/nakkeslengprosjekt/asdf/build /home/nakkeslengprosjekt/asdf/build/CMakeFiles/examples_common.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/examples_common.dir/depend

