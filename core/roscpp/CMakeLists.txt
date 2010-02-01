cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

# Is there a way to get this information from rosconsole, which is where it
# matters?
find_path(log4cxx_include_dir log4cxx/logger.h)
if(log4cxx_include_dir STREQUAL "log4cxx_include_dir-NOTFOUND")
  message(FATAL_ERROR "Could not find log4cxx/logger.h. Set CMAKE_INCLUDE_PATH.")
endif(log4cxx_include_dir STREQUAL "log4cxx_include_dir-NOTFOUND")
include_directories(${log4cxx_include_dir})

rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()
add_subdirectory(src)
add_subdirectory(test EXCLUDE_FROM_ALL)
