# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.13.4/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.13.4/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/src"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build"

# Include any dependencies generated for this target.
include Core/tools/mtm/CMakeFiles/mtm.dir/depend.make

# Include the progress variables for this target.
include Core/tools/mtm/CMakeFiles/mtm.dir/progress.make

# Include the compile flags for this target's objects.
include Core/tools/mtm/CMakeFiles/mtm.dir/flags.make

Core/tools/mtm/CMakeFiles/mtm.dir/mtm.cpp.o: Core/tools/mtm/CMakeFiles/mtm.dir/flags.make
Core/tools/mtm/CMakeFiles/mtm.dir/mtm.cpp.o: /Users/steffen/OneDrive\ -\ Nexus365/Code/Mobile\ Robotics/codebase/moos/src/Core/tools/mtm/mtm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Core/tools/mtm/CMakeFiles/mtm.dir/mtm.cpp.o"
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/mtm" && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mtm.dir/mtm.cpp.o -c "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/src/Core/tools/mtm/mtm.cpp"

Core/tools/mtm/CMakeFiles/mtm.dir/mtm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mtm.dir/mtm.cpp.i"
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/mtm" && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/src/Core/tools/mtm/mtm.cpp" > CMakeFiles/mtm.dir/mtm.cpp.i

Core/tools/mtm/CMakeFiles/mtm.dir/mtm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mtm.dir/mtm.cpp.s"
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/mtm" && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/src/Core/tools/mtm/mtm.cpp" -o CMakeFiles/mtm.dir/mtm.cpp.s

# Object files for target mtm
mtm_OBJECTS = \
"CMakeFiles/mtm.dir/mtm.cpp.o"

# External object files for target mtm
mtm_EXTERNAL_OBJECTS =

bin/mtm: Core/tools/mtm/CMakeFiles/mtm.dir/mtm.cpp.o
bin/mtm: Core/tools/mtm/CMakeFiles/mtm.dir/build.make
bin/mtm: lib/libMOOS.a
bin/mtm: Core/tools/mtm/CMakeFiles/mtm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/mtm"
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/mtm" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mtm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Core/tools/mtm/CMakeFiles/mtm.dir/build: bin/mtm

.PHONY : Core/tools/mtm/CMakeFiles/mtm.dir/build

Core/tools/mtm/CMakeFiles/mtm.dir/clean:
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/mtm" && $(CMAKE_COMMAND) -P CMakeFiles/mtm.dir/cmake_clean.cmake
.PHONY : Core/tools/mtm/CMakeFiles/mtm.dir/clean

Core/tools/mtm/CMakeFiles/mtm.dir/depend:
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/src" "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/src/Core/tools/mtm" "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build" "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/mtm" "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/mtm/CMakeFiles/mtm.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : Core/tools/mtm/CMakeFiles/mtm.dir/depend

