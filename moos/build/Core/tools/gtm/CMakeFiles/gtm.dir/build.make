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
include Core/tools/gtm/CMakeFiles/gtm.dir/depend.make

# Include the progress variables for this target.
include Core/tools/gtm/CMakeFiles/gtm.dir/progress.make

# Include the compile flags for this target's objects.
include Core/tools/gtm/CMakeFiles/gtm.dir/flags.make

Core/tools/gtm/CMakeFiles/gtm.dir/gtm.cpp.o: Core/tools/gtm/CMakeFiles/gtm.dir/flags.make
Core/tools/gtm/CMakeFiles/gtm.dir/gtm.cpp.o: /Users/steffen/OneDrive\ -\ Nexus365/Code/Mobile\ Robotics/codebase/moos/src/Core/tools/gtm/gtm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Core/tools/gtm/CMakeFiles/gtm.dir/gtm.cpp.o"
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/gtm" && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gtm.dir/gtm.cpp.o -c "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/src/Core/tools/gtm/gtm.cpp"

Core/tools/gtm/CMakeFiles/gtm.dir/gtm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gtm.dir/gtm.cpp.i"
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/gtm" && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/src/Core/tools/gtm/gtm.cpp" > CMakeFiles/gtm.dir/gtm.cpp.i

Core/tools/gtm/CMakeFiles/gtm.dir/gtm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gtm.dir/gtm.cpp.s"
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/gtm" && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/src/Core/tools/gtm/gtm.cpp" -o CMakeFiles/gtm.dir/gtm.cpp.s

# Object files for target gtm
gtm_OBJECTS = \
"CMakeFiles/gtm.dir/gtm.cpp.o"

# External object files for target gtm
gtm_EXTERNAL_OBJECTS =

bin/gtm: Core/tools/gtm/CMakeFiles/gtm.dir/gtm.cpp.o
bin/gtm: Core/tools/gtm/CMakeFiles/gtm.dir/build.make
bin/gtm: lib/libMOOS.a
bin/gtm: Core/tools/gtm/CMakeFiles/gtm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/gtm"
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/gtm" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gtm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Core/tools/gtm/CMakeFiles/gtm.dir/build: bin/gtm

.PHONY : Core/tools/gtm/CMakeFiles/gtm.dir/build

Core/tools/gtm/CMakeFiles/gtm.dir/clean:
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/gtm" && $(CMAKE_COMMAND) -P CMakeFiles/gtm.dir/cmake_clean.cmake
.PHONY : Core/tools/gtm/CMakeFiles/gtm.dir/clean

Core/tools/gtm/CMakeFiles/gtm.dir/depend:
	cd "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/src" "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/src/Core/tools/gtm" "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build" "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/gtm" "/Users/steffen/OneDrive - Nexus365/Code/Mobile Robotics/codebase/moos/build/Core/tools/gtm/CMakeFiles/gtm.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : Core/tools/gtm/CMakeFiles/gtm.dir/depend

