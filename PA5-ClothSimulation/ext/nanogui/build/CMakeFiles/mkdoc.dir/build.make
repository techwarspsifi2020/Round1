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
CMAKE_SOURCE_DIR = "/home/ahmad/Documents/LUMS/Fall 3/CG/PA5-ClothSimulation/ext/nanogui"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/ahmad/Documents/LUMS/Fall 3/CG/PA5-ClothSimulation/ext/nanogui/build"

# Utility rule file for mkdoc.

# Include the progress variables for this target.
include CMakeFiles/mkdoc.dir/progress.make

CMakeFiles/mkdoc:
	python3 /home/ahmad/Documents/LUMS/Fall\ 3/CG/PA5-ClothSimulation/ext/nanogui/docs/mkdoc_rst.py -fvisibility=hidden -Wall -Wextra -std=c++14 -flto -fno-fat-lto-objects -I/home/ahmad/Documents/LUMS/Fall\ 3/CG/PA5-ClothSimulation/ext/nanogui/ext/eigen -I/home/ahmad/Documents/LUMS/Fall\ 3/CG/PA5-ClothSimulation/ext/nanogui/ext/glfw/include -I/home/ahmad/Documents/LUMS/Fall\ 3/CG/PA5-ClothSimulation/ext/nanogui/ext/nanovg/src -I/home/ahmad/Documents/LUMS/Fall\ 3/CG/PA5-ClothSimulation/ext/nanogui/include -I/home/ahmad/Documents/LUMS/Fall\ 3/CG/PA5-ClothSimulation/ext/nanogui/build -I/home/ahmad/Documents/LUMS/Fall\ 3/CG/PA5-ClothSimulation/ext/nanogui/build -I/home/ahmad/Documents/LUMS/Fall\ 3/CG/PA5-ClothSimulation/ext/nanogui/ext/coro -I/home/ahmad/Documents/LUMS/Fall\ 3/CG/PA5-ClothSimulation/ext/nanogui/ext/pybind11/include -I/usr/include/python3.5m -DCORO_SJLJ -DNANOGUI_PYTHON -DNANOGUI_SHARED -DNVG_SHARED -DGLAD_GLAPI_EXPORT -DDOXYGEN_DOCUMENTATION_BUILD /home/ahmad/Documents/LUMS/Fall\ 3/CG/PA5-ClothSimulation/ext/nanogui/include/nanogui/*.h > /home/ahmad/Documents/LUMS/Fall\ 3/CG/PA5-ClothSimulation/ext/nanogui/python/py_doc.h

mkdoc: CMakeFiles/mkdoc
mkdoc: CMakeFiles/mkdoc.dir/build.make

.PHONY : mkdoc

# Rule to build all files generated by this target.
CMakeFiles/mkdoc.dir/build: mkdoc

.PHONY : CMakeFiles/mkdoc.dir/build

CMakeFiles/mkdoc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mkdoc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mkdoc.dir/clean

CMakeFiles/mkdoc.dir/depend:
	cd "/home/ahmad/Documents/LUMS/Fall 3/CG/PA5-ClothSimulation/ext/nanogui/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/ahmad/Documents/LUMS/Fall 3/CG/PA5-ClothSimulation/ext/nanogui" "/home/ahmad/Documents/LUMS/Fall 3/CG/PA5-ClothSimulation/ext/nanogui" "/home/ahmad/Documents/LUMS/Fall 3/CG/PA5-ClothSimulation/ext/nanogui/build" "/home/ahmad/Documents/LUMS/Fall 3/CG/PA5-ClothSimulation/ext/nanogui/build" "/home/ahmad/Documents/LUMS/Fall 3/CG/PA5-ClothSimulation/ext/nanogui/build/CMakeFiles/mkdoc.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/mkdoc.dir/depend
