#!/usr/bin/env python

import inspect
from roslib.rosshutil import *
import sys


def all():
    if sys.platform == 'win32':
        win32_generator_flag = '-G "NMake Makefiles"'
    else:
        win32_generator_flag = ''
    mkdir('build', make_parents=True)
    mkdir('bin', make_parents=True)
    ret_code = run_process('cmake %s ..' % win32_generator_flag,
        msg_on_fail='[rosbuild] CMake failed; trying to clean and start over',
        working_dir='build')[0]
    if ret_code != 0:
        # Failed, try again after a scrub
        clean()
        mkdir('build', make_parents=True)
        run_process('cmake %s ..' % win32_generator_flag,
                    working_dir='build')
    return run_process('${MAKE} %s' % envvar('PARALLEL_JOBS'),
                       working_dir='build')[0]


def install():
    all()
    return run_process('${MAKE} install', working_dir='build')[0]


def clean():
    run_process('${MAKE} clean', working_dir='build')
    return rmdir('build', recurse=True)


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
        all()
    else:
        for target in argv:
            if target not in targets:
                print >>sys.stderr, 'No such target: ' + target
                return 1
            if targets[target]():
                return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

