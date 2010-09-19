cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_gensrv()

rosbuild_add_library(topic_tools src/shape_shifter.cpp src/parse.cpp)

set(EXECUTABLE_OUTPUT_PATH bin)

rosbuild_add_executable(switch_mux src/switch_mux.cpp)
target_link_libraries(switch_mux topic_tools)

rosbuild_add_executable(mux src/mux.cpp)
target_link_libraries(mux topic_tools)

rosbuild_add_executable(relay src/relay.cpp)
target_link_libraries(relay topic_tools)

rosbuild_add_executable(drop src/drop.cpp)
target_link_libraries(drop topic_tools)

#rosbuild_add_executable(demux demux.cpp)
#target_link_libraries(demux topic_tools)

rosbuild_add_executable(throttle src/throttle.cpp)
target_link_libraries(throttle topic_tools)

rosbuild_add_gtest(utest test/utest.cpp)
target_link_libraries(utest topic_tools)
set_target_properties(utest PROPERTIES RUNTIME_OUTPUT_DIRECTORY test)

rosbuild_add_executable(test_shapeshifter test/test_shapeshifter.cpp)
rosbuild_add_gtest_build_flags(test_shapeshifter)
target_link_libraries(test_shapeshifter topic_tools)
set_target_properties(test_shapeshifter PROPERTIES RUNTIME_OUTPUT_DIRECTORY test)

rosbuild_add_rostest(test/shapeshifter.test)
rosbuild_add_rostest(test/throttle.test)
rosbuild_add_rostest(test/drop.test)
rosbuild_add_rostest(test/relay.test)
rosbuild_add_rostest(test/mux.test)
rosbuild_add_rostest(test/switch_mux.test)
rosbuild_add_rostest(test/switch_mux_leading_slash.test)
rosbuild_add_rostest(test/switch_mux_none.test)
rosbuild_add_rostest(test/mux_add.test)

rosbuild_add_pyunit(test/args.py)

rosbuild_install_library(topic_tools)
rosbuild_install_directory(scripts)
rosbuild_install_directory(include)

rosbuild_install_executable(switch_mux)
rosbuild_install_executable(mux)
rosbuild_install_executable(relay)
rosbuild_install_executable(drop)
rosbuild_install_executable(throttle)
