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
CMAKE_SOURCE_DIR = /home/jetson/cloud_edge_test/Keyframe

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jetson/cloud_edge_test/Keyframe/build

# Include any dependencies generated for this target.
include CMakeFiles/edge.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/edge.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/edge.dir/flags.make

CMakeFiles/edge.dir/edge.cpp.o: CMakeFiles/edge.dir/flags.make
CMakeFiles/edge.dir/edge.cpp.o: ../edge.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/cloud_edge_test/Keyframe/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/edge.dir/edge.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/edge.dir/edge.cpp.o -c /home/jetson/cloud_edge_test/Keyframe/edge.cpp

CMakeFiles/edge.dir/edge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/edge.dir/edge.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/cloud_edge_test/Keyframe/edge.cpp > CMakeFiles/edge.dir/edge.cpp.i

CMakeFiles/edge.dir/edge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/edge.dir/edge.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/cloud_edge_test/Keyframe/edge.cpp -o CMakeFiles/edge.dir/edge.cpp.s

# Object files for target edge
edge_OBJECTS = \
"CMakeFiles/edge.dir/edge.cpp.o"

# External object files for target edge
edge_EXTERNAL_OBJECTS =

edge: CMakeFiles/edge.dir/edge.cpp.o
edge: CMakeFiles/edge.dir/build.make
edge: /usr/lib/aarch64-linux-gnu/libboost_serialization.so.1.71.0
edge: /home/jetson/opencv4/lib/libopencv_dnn.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_gapi.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_highgui.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_ml.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_objdetect.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_photo.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_stitching.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_video.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_videoio.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_imgcodecs.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_calib3d.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_features2d.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_flann.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_imgproc.so.4.2.0
edge: /home/jetson/opencv4/lib/libopencv_core.so.4.2.0
edge: CMakeFiles/edge.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jetson/cloud_edge_test/Keyframe/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable edge"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/edge.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/edge.dir/build: edge

.PHONY : CMakeFiles/edge.dir/build

CMakeFiles/edge.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/edge.dir/cmake_clean.cmake
.PHONY : CMakeFiles/edge.dir/clean

CMakeFiles/edge.dir/depend:
	cd /home/jetson/cloud_edge_test/Keyframe/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jetson/cloud_edge_test/Keyframe /home/jetson/cloud_edge_test/Keyframe /home/jetson/cloud_edge_test/Keyframe/build /home/jetson/cloud_edge_test/Keyframe/build /home/jetson/cloud_edge_test/Keyframe/build/CMakeFiles/edge.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/edge.dir/depend

