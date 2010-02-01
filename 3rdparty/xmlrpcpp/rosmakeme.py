#!/usr/bin/env python

import inspect
from os import getcwd
from os.path import basename, join, sep
from roslib.rosshutil import *
import sys

EXTRA_CMAKE_FLAGS = ''
PACKAGE_NAME = basename(getcwd().rstrip(sep))


def all():
    rosbuild_dir = run_process('rospack find rosbuild',
                               communicate=True)[1].strip()
    toolchain_file = join(rosbuild_dir, 'rostoolchain.cmake')
    CMAKE_FLAGS = '-Wdev -DCMAKE_TOOLCHAIN_FILE=%s '%toolchain_file + \
            EXTRA_CMAKE_FLAGS
    if sys.platform == 'win32':
        win32_generator_flag = '-G "NMake Makefiles"'
    else:
        win32_generator_flag = ''
    mkdir('build', make_parents=True)
    mkdir('bin', make_parents=True)
    ret_code = run_process('cmake %s %s ..' % (win32_generator_flag,
                                               CMAKE_FLAGS),
                           working_dir='build')[0]
    return run_process('${MAKE} %s' % envvar('ROS_PARALLEL_JOBS'),
                       working_dir='build')[0]


def clean():
    run_process('${MAKE} clean', working_dir='build', exit_on_fail=1)
    for ii in ['msg/cpp', 'msg/lisp', 'msg/oct', 'msg/java', 'srv/cpp',
               'srv/lisp', 'srv/oct', 'srv/java', 'src/%s/msg'%PACKAGE_NAME,
               'src/%s/srv'%PACKAGE_NAME]:
        rm(ii, recurse=True)
    return rm('build', recurse=True)


def test():
    all()
    ret_code = run_process('${MAKE} -k test', working_dir='build')
    if ret_code == 0:
        return run_process('${MAKE} test-results', working_dir='build')[0]
    else:
        run_process('${MAKE} test-results', working_dir='build')
        return 1


def tests():
    all()
    return run_process('${MAKE} tests', working_dir='build')[0]


def test_future():
    all()
    return run_process('${MAKE} -k test-future', working_dir='build')[0]


def gcoverage():
    all()
    return run_process('${MAKE} gcoverage', working_dir='build')[0]


def main(argv):
    if argv[0] == 'rosmakeme.py':
        argv = argv[1:]
    targets = dict(inspect.getmembers(sys.modules[__name__],
                                      inspect.isfunction))
    if not argv:
        return all()
    for target in argv:
        if target not in targets:
            print >>sys.stderr, 'No such target: ' + target
            return 1
        if targets[target]():
            return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

