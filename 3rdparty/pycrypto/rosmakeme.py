#!/usr/bin/env python

import inspect
from roslib.rosshutil import *
from os import environ, getcwd
from os.path import exists, isdir, join
import sys


tarball = join('build', 'pycrypto-2.0.1.tar.gz')
tarball_url = 'http://pr.willowgarage.com/downloads/pycrypto-2.0.1.tar.gz'
source_dir = join('build', 'pycrypto-2.0.1')
md5sum_file = 'pycrypto-2.0.1.tar.gz.md5sum'


def all():
    if exists('pycrypto'):
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
    run_process('tar xzf %s' % tarball, working_dir='build')

    # Move the file contents somewhere a tad more useful
    rm('src', recurse=True)
    mkdir('src')
    run_process('python setup.py build', working_dir=source_dir)
    mv(join(source_dir, 'build', 'lib.*', 'Crypto'), 'src')
    touch('pycrypto')


def clean():
    rm('src', recurse=True)
    rm('pycrypto', recurse=True)
    rm(SOURCE_DIR, recurse=True)


def wipe():
    clean()
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


