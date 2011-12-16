cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)

# Init
rosbuild_init()

# Cmake settings
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Add boost
rosbuild_add_boost_directories()

# Create library
rosbuild_add_library(teleop_framework src/teleop_common.cpp src/teleop_source_adapter.cpp)
rosbuild_link_boost(teleop_framework thread)
