# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project

# Include any dependencies generated for this target.
include vecmath/CMakeFiles/vecmath.dir/depend.make

# Include the progress variables for this target.
include vecmath/CMakeFiles/vecmath.dir/progress.make

# Include the compile flags for this target's objects.
include vecmath/CMakeFiles/vecmath.dir/flags.make

vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o: vecmath/Matrix2f.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/vecmath.dir/Matrix2f.cpp.o -c /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Matrix2f.cpp

vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Matrix2f.cpp.i"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Matrix2f.cpp > CMakeFiles/vecmath.dir/Matrix2f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Matrix2f.cpp.s"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Matrix2f.cpp -o CMakeFiles/vecmath.dir/Matrix2f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o.requires:
.PHONY : vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o.requires

vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o.provides: vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o.requires
	$(MAKE) -f vecmath/CMakeFiles/vecmath.dir/build.make vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o.provides.build
.PHONY : vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o.provides

vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o.provides.build: vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o

vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o: vecmath/Matrix3f.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/vecmath.dir/Matrix3f.cpp.o -c /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Matrix3f.cpp

vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Matrix3f.cpp.i"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Matrix3f.cpp > CMakeFiles/vecmath.dir/Matrix3f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Matrix3f.cpp.s"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Matrix3f.cpp -o CMakeFiles/vecmath.dir/Matrix3f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o.requires:
.PHONY : vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o.requires

vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o.provides: vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o.requires
	$(MAKE) -f vecmath/CMakeFiles/vecmath.dir/build.make vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o.provides.build
.PHONY : vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o.provides

vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o.provides.build: vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o

vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o: vecmath/Matrix4f.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/vecmath.dir/Matrix4f.cpp.o -c /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Matrix4f.cpp

vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Matrix4f.cpp.i"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Matrix4f.cpp > CMakeFiles/vecmath.dir/Matrix4f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Matrix4f.cpp.s"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Matrix4f.cpp -o CMakeFiles/vecmath.dir/Matrix4f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o.requires:
.PHONY : vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o.requires

vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o.provides: vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o.requires
	$(MAKE) -f vecmath/CMakeFiles/vecmath.dir/build.make vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o.provides.build
.PHONY : vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o.provides

vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o.provides.build: vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o

vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o: vecmath/Quat4f.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/vecmath.dir/Quat4f.cpp.o -c /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Quat4f.cpp

vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Quat4f.cpp.i"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Quat4f.cpp > CMakeFiles/vecmath.dir/Quat4f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Quat4f.cpp.s"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Quat4f.cpp -o CMakeFiles/vecmath.dir/Quat4f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o.requires:
.PHONY : vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o.requires

vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o.provides: vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o.requires
	$(MAKE) -f vecmath/CMakeFiles/vecmath.dir/build.make vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o.provides.build
.PHONY : vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o.provides

vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o.provides.build: vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o

vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o: vecmath/Vector2f.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/vecmath.dir/Vector2f.cpp.o -c /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Vector2f.cpp

vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Vector2f.cpp.i"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Vector2f.cpp > CMakeFiles/vecmath.dir/Vector2f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Vector2f.cpp.s"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Vector2f.cpp -o CMakeFiles/vecmath.dir/Vector2f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o.requires:
.PHONY : vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o.requires

vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o.provides: vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o.requires
	$(MAKE) -f vecmath/CMakeFiles/vecmath.dir/build.make vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o.provides.build
.PHONY : vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o.provides

vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o.provides.build: vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o

vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o: vecmath/Vector3f.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/vecmath.dir/Vector3f.cpp.o -c /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Vector3f.cpp

vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Vector3f.cpp.i"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Vector3f.cpp > CMakeFiles/vecmath.dir/Vector3f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Vector3f.cpp.s"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Vector3f.cpp -o CMakeFiles/vecmath.dir/Vector3f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o.requires:
.PHONY : vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o.requires

vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o.provides: vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o.requires
	$(MAKE) -f vecmath/CMakeFiles/vecmath.dir/build.make vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o.provides.build
.PHONY : vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o.provides

vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o.provides.build: vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o

vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o: vecmath/CMakeFiles/vecmath.dir/flags.make
vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o: vecmath/Vector4f.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/vecmath.dir/Vector4f.cpp.o -c /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Vector4f.cpp

vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vecmath.dir/Vector4f.cpp.i"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Vector4f.cpp > CMakeFiles/vecmath.dir/Vector4f.cpp.i

vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vecmath.dir/Vector4f.cpp.s"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/Vector4f.cpp -o CMakeFiles/vecmath.dir/Vector4f.cpp.s

vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o.requires:
.PHONY : vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o.requires

vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o.provides: vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o.requires
	$(MAKE) -f vecmath/CMakeFiles/vecmath.dir/build.make vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o.provides.build
.PHONY : vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o.provides

vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o.provides.build: vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o

# Object files for target vecmath
vecmath_OBJECTS = \
"CMakeFiles/vecmath.dir/Matrix2f.cpp.o" \
"CMakeFiles/vecmath.dir/Matrix3f.cpp.o" \
"CMakeFiles/vecmath.dir/Matrix4f.cpp.o" \
"CMakeFiles/vecmath.dir/Quat4f.cpp.o" \
"CMakeFiles/vecmath.dir/Vector2f.cpp.o" \
"CMakeFiles/vecmath.dir/Vector3f.cpp.o" \
"CMakeFiles/vecmath.dir/Vector4f.cpp.o"

# External object files for target vecmath
vecmath_EXTERNAL_OBJECTS =

vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/build.make
vecmath/libvecmath.a: vecmath/CMakeFiles/vecmath.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libvecmath.a"
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && $(CMAKE_COMMAND) -P CMakeFiles/vecmath.dir/cmake_clean_target.cmake
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vecmath.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vecmath/CMakeFiles/vecmath.dir/build: vecmath/libvecmath.a
.PHONY : vecmath/CMakeFiles/vecmath.dir/build

vecmath/CMakeFiles/vecmath.dir/requires: vecmath/CMakeFiles/vecmath.dir/Matrix2f.cpp.o.requires
vecmath/CMakeFiles/vecmath.dir/requires: vecmath/CMakeFiles/vecmath.dir/Matrix3f.cpp.o.requires
vecmath/CMakeFiles/vecmath.dir/requires: vecmath/CMakeFiles/vecmath.dir/Matrix4f.cpp.o.requires
vecmath/CMakeFiles/vecmath.dir/requires: vecmath/CMakeFiles/vecmath.dir/Quat4f.cpp.o.requires
vecmath/CMakeFiles/vecmath.dir/requires: vecmath/CMakeFiles/vecmath.dir/Vector2f.cpp.o.requires
vecmath/CMakeFiles/vecmath.dir/requires: vecmath/CMakeFiles/vecmath.dir/Vector3f.cpp.o.requires
vecmath/CMakeFiles/vecmath.dir/requires: vecmath/CMakeFiles/vecmath.dir/Vector4f.cpp.o.requires
.PHONY : vecmath/CMakeFiles/vecmath.dir/requires

vecmath/CMakeFiles/vecmath.dir/clean:
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath && $(CMAKE_COMMAND) -P CMakeFiles/vecmath.dir/cmake_clean.cmake
.PHONY : vecmath/CMakeFiles/vecmath.dir/clean

vecmath/CMakeFiles/vecmath.dir/depend:
	cd /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath /afs/athena.mit.edu/user/x/t/xtnbui/Desktop/6.837/6.837_final_project/vecmath/CMakeFiles/vecmath.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vecmath/CMakeFiles/vecmath.dir/depend

