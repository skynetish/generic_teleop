cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)

# Init
rosbuild_init()

# Cmake settings
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Add boost
rosbuild_add_boost_directories()

# Create executables
rosbuild_add_executable(teleop_source_ros src/teleop_source_ros.cpp)
rosbuild_link_boost(teleop_source_ros thread)
rosbuild_add_executable(teleop_sink_twist_ros src/teleop_sink_twist_ros.cpp)
rosbuild_link_boost(teleop_sink_twist_ros thread)
