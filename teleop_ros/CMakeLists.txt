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
rosbuild_add_executable(
  teleop_source_node
  src/teleop_source_node.cpp
  src/teleop_source_node_main.cpp
)
rosbuild_link_boost(teleop_source_node thread)
rosbuild_add_executable(
  teleop_sink_twist_node
  src/teleop_sink_twist_node.cpp
  src/teleop_sink_twist_node_main.cpp
)
rosbuild_link_boost(teleop_sink_twist_node thread)
