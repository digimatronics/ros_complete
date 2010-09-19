cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_install_directory(cmake)
rosbuild_install_directory(scripts)
rosbuild_install_directory(src/rospy)
