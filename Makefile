
all:
	@if [ ! $(ROS_ROOT) ]; then echo "Please set ROS_ROOT first"; false; fi
	cd tools/rospack && make
	@if test -z `which rospack`; then echo "Please add ROS_ROOT/bin to PATH"; false; fi
	cd tools/rosdep && make
	cd 3rdparty/gtest && make
	cd core/genmsg_cpp && make
	cd core/roslib && make
	cd core/rospy && make
	cd 3rdparty/pycrypto && make
	cd 3rdparty/paramiko && make
	cd 3rdparty/xmlrpcpp && make
	cd tools/roslaunch && make
	cd test/rostest && make
	cd core/rosconsole && make
	cd core/roscpp && make
	cd core/rosout && make
	@echo "\nHOORAY! The ROS tools are now built. Now, you can use 'rosmake' for\nrecursive builds.  For example, try\n    rosmake roscpp_tutorials\n"

clean:
	@if test -z `which rospack`; then echo "It appears that you have already done a 'make clean' because rospack is gone."; false; fi
	make -C core/genmsg_cpp clean
	make -C core/roslib clean
	make -C core/rospy clean
	make -C 3rdparty/pycrypto clean
	make -C 3rdparty/paramiko clean
	make -C 3rdparty/xmlrpcpp clean
	make -C core/rosconsole clean
	make -C core/roscpp clean
	make -C core/rosout clean
	make -C 3rdparty/gtest clean
	make -C tools/rosdep clean
	make -C test/rostest clean
	make -C tools/roslaunch clean
	rm -f `find . -name *.pyc`

clean-everything:
	cd tools/rospack && make
	@for i in `rospack list-names` ; do  if [ $$i = rospack ] ; then continue; fi; echo "cleaning $$i"; cd `rospack find $$i` && make clean; done
	cd tools/rospack && make clean

wipe-everything:
	cd tools/rospack && make
	@for i in `rospack list-names` ; do  if [ $$i = rospack ] ; then continue; fi; echo "wiping $$i"; cd `rospack find $$i` && make wipe; done
	rm -f rosmakeall-* stderr.txt stdout.txt rosmakeall-profile build-failure test-failure
	cd tools/rospack && make clean

minimal:
	cd tools/rospack && make
	cd tools/rosdep && make
	cd 3rdparty/gtest && make
	cd 3rdparty/pycrypto && make
	cd 3rdparty/paramiko && make
	cd tools/roslaunch && make
	cd test/rostest && make
	rosmake -v genmsg_cpp

### copied below since it can't be found before being built include $(shell rospack find mk)/cmake_stack.mk

# set EXTRA_CMAKE_FLAGS in the including Makefile in order to add tweaks
#CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)
CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=../core/rosbuild/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)

# The all target does the heavy lifting, creating the build directory and
# invoking CMake
all_dist:
	@mkdir -p build
	-mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) ..
	#cd build && make $(ROS_PARALLEL_JOBS)

# The clean target blows everything away
# It also removes auto-generated message/service code directories, 
# to handle the case where the original .msg/.srv file has been removed,
# and thus CMake no longer knows about it.
clean_dist:
	-cd build && make clean
	#rm -rf msg/cpp msg/lisp msg/oct msg/java srv/cpp srv/lisp srv/oct srv/java src/$(PACKAGE_NAME)/msg src/$(PACKAGE_NAME)/srv
	rm -rf build
	#rm -f .build-version

# Run the script that does the build, then do a fairly hacky cleanup, #1598
package_source: all_dist
	`rospack find rosbuild`/bin/makestackdist $(CURDIR)
	find build -mindepth 1 -not -name "*.bz2" | xargs rm -rf
	rm -rf bin
