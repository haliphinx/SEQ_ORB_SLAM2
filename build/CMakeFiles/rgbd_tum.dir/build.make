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
CMAKE_SOURCE_DIR = /home/xhu/Code/ORB_SLAM2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xhu/Code/ORB_SLAM2/build

# Include any dependencies generated for this target.
include CMakeFiles/rgbd_tum.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rgbd_tum.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rgbd_tum.dir/flags.make

CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o: CMakeFiles/rgbd_tum.dir/flags.make
CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o: ../Examples/RGB-D/rgbd_tum.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xhu/Code/ORB_SLAM2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o -c /home/xhu/Code/ORB_SLAM2/Examples/RGB-D/rgbd_tum.cc

CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xhu/Code/ORB_SLAM2/Examples/RGB-D/rgbd_tum.cc > CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.i

CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xhu/Code/ORB_SLAM2/Examples/RGB-D/rgbd_tum.cc -o CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.s

CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o.requires:

.PHONY : CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o.requires

CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o.provides: CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o.requires
	$(MAKE) -f CMakeFiles/rgbd_tum.dir/build.make CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o.provides.build
.PHONY : CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o.provides

CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o.provides.build: CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o


# Object files for target rgbd_tum
rgbd_tum_OBJECTS = \
"CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o"

# External object files for target rgbd_tum
rgbd_tum_EXTERNAL_OBJECTS =

../Examples/RGB-D/rgbd_tum: CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o
../Examples/RGB-D/rgbd_tum: CMakeFiles/rgbd_tum.dir/build.make
../Examples/RGB-D/rgbd_tum: ../lib/libORB_SLAM2.so
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_stitching.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_superres.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_videostab.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_surface_matching.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_optflow.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_ximgproc.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_bgsegm.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_fuzzy.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_xphoto.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_bioinspired.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_sfm.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_xfeatures2d.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_shape.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_dpm.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_xobjdetect.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_hfs.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_reg.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_hdf.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_stereo.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_aruco.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_freetype.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_img_hash.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_line_descriptor.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_rgbd.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_structured_light.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_face.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_photo.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_objdetect.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_cvv.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_saliency.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_tracking.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_video.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_plot.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_datasets.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_text.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_ml.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_dnn.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_ccalib.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_calib3d.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_features2d.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_flann.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_highgui.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_videoio.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_imgcodecs.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_imgproc.so.3.4.5
../Examples/RGB-D/rgbd_tum: /usr/local/lib/libopencv_core.so.3.4.5
../Examples/RGB-D/rgbd_tum: /home/xhu/Code/Pangolin/build/src/libpangolin.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libdc1394.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libpng.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libz.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libjpeg.so
../Examples/RGB-D/rgbd_tum: /usr/lib/x86_64-linux-gnu/libtiff.so
../Examples/RGB-D/rgbd_tum: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/RGB-D/rgbd_tum: ../Thirdparty/g2o/lib/libg2o.so
../Examples/RGB-D/rgbd_tum: CMakeFiles/rgbd_tum.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xhu/Code/ORB_SLAM2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/RGB-D/rgbd_tum"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rgbd_tum.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rgbd_tum.dir/build: ../Examples/RGB-D/rgbd_tum

.PHONY : CMakeFiles/rgbd_tum.dir/build

CMakeFiles/rgbd_tum.dir/requires: CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o.requires

.PHONY : CMakeFiles/rgbd_tum.dir/requires

CMakeFiles/rgbd_tum.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rgbd_tum.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rgbd_tum.dir/clean

CMakeFiles/rgbd_tum.dir/depend:
	cd /home/xhu/Code/ORB_SLAM2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xhu/Code/ORB_SLAM2 /home/xhu/Code/ORB_SLAM2 /home/xhu/Code/ORB_SLAM2/build /home/xhu/Code/ORB_SLAM2/build /home/xhu/Code/ORB_SLAM2/build/CMakeFiles/rgbd_tum.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rgbd_tum.dir/depend

