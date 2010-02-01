#!/usr/bin/env python

import inspect
from roslib.rosshutil import *
from os import environ, getcwd
from os.path import exists, isdir, join
import sys


if sys.platform == 'win32':
    tarball = join('build', 'gtest-1.4.0.zip')
    tarball_url = 'http://googletest.googlecode.com/files/gtest-1.4.0.zip'
    source_dir = join('build', 'gtest-1.4.0')
else:
    tarball = join('build', 'gtest-1.0.1.tar.gz')
    tarball_url = 'http://pr.willowgarage.com/downloads/gtest-1.0.1.tar.gz'
    source_dir = join('build', 'gtest-1.0.1')
md5sum_file = 'gtest-1.0.1.tar.gz.md5sum'


def all():
    if exists('gtest'):
        return
    # Download the file
    mkdir('build', make_parents=True)
    rosbuild_dir = run_process('rospack find rosbuild',
                               communicate=True)[1].strip()
    if which('awk'):
        md5 = run_process("awk {'print $1'} %s" % md5sum_file)[1]
    else:
        md5 = ''
    run_process('python %s %s %s %s' % (join(rosbuild_dir, 'bin',
                                             'download_checkmd5.py'),
                                        tarball_url, tarball, md5))
    # Extract the file
    if sys.platform == 'win32':
        if not isdir(source_dir):
            import zipfile
            archive = zipfile.ZipFile(tarball)
            # Big, evil security risk!
            archive.extractall(path='build\\')
            archive.close()
    else:
        run_process('tar xzf %s' % tarball, working_dir='build')
    # Build
    if sys.platform == 'win32':
        run_process('devenv msvc\\gtest.sln /build "Release|Win32"',
                working_dir=source_dir)
    else:
        # gtest's death test appears to hang when gtest is compiled with
        # Bullseye's gcc wrapper.  So, if COVFILE is set (which indicates that
        # we're doing a coverage build), then we heuristically modify the PATH
        # to get at the real gcc.
        extra_env = {}
        if 'COVFILE' in environ:
            extra_env['PATH'] = '/usr/bin:' + environ.get('PATH', '')
        run_process('configure --prefix=%s/gtest' % getcwd(),
                extra_env=extra_env, working_dir=source_dir)
        run_process('make install', extra_env=extra_env,
                working_dir=source_dir)
    touch('gtest')


def clean():
    rm('gtest', recurse=True)
    rm(SOURCE_DIR, recurse=True)


def wipe():
    rm('build', recurse=True)


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


