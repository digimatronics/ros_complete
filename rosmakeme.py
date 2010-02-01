#!/usr/bin/env python

import inspect
from roslib.rosshutil import *
import sys


def core_tools():
    run_process('rosmake --rosdep-install --bootstrap')
    print >>sys.stderr, 'You have built the minimal set of ROS tools.'
    print >>sys.stderr, 'If you want to make all ros tools type "rosmake ros".'
    print >>sys.stderr, 'Or you can rosmake any other package in your \
ROS_PACKAGE_PATH.'
    return 0


def clean():
    is_executable('rospack',
                  msg_on_fail="It appears that you have already done a 'make \
clean' because rospack is gone.",
                  exit_on_fail=1)
    return run_process('rosmake -r --target=clean ros')


cmake_flags = '-Wdev -DCMAKE_TOOLCHAIN_FILE=./core/rosbuild/rostoolchain.cmake'
def all_dist():
    core_tools()
    mkdir('build', make_parents=True)
    mkdir('bin', make_parents=True)
    return run_process('cmake %s ..' % cmake_flags, working_dir='build')[0]


def clean_dist():
    run_process('python dostuff.py clean',
                working_dir='build')
    return rmdir('build', recurse=True)


def package_source():
    all_dist()
    rosbuild_dir = eval('rospack find rosbuild')
    run_process('%s/bin/makestackdist %s' % (rosbuild_dir, curdir()))
    for dir in find(root='build', min_depth=1, exclude_names='*.bz2'):
        rmdir(dir, recurse=True)
    return rmdir('bin', recurse=True)


def main(argv):
    if argv[0] == 'rosmakeme.py':
        argv = argv[1:]
    targets = dict(inspect.getmembers(sys.modules[__name__],
                                      inspect.isfunction))

    for target in argv:
        if target == 'minimal':
            return core_tools()
        if target not in targets:
            print >>sys.stderr, 'No such target: ' + target
            return 1
        if targets[target]():
            return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

