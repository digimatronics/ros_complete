#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import os
import sys

import roslib.rosenv
import roslib.packages

def print_warning(msg):
    """print warning to screen (bold red)"""
    print >> sys.stderr, '\033[31m%s\033[0m'%msg
    
def on_ros_path(p):
    """
    @param p: path
    @type  p: str
    @return: True if p is on the ROS path (ROS_ROOT, ROS_PACKAGE_PATH)
    """
    pkg = os.path.realpath(roslib.rosenv.resolve_path(p))
    paths = [p for p in roslib.packages.get_package_paths()]
    paths = [os.path.realpath(roslib.rosenv.resolve_path(x)) for x in paths]
    return bool([x for x in paths if pkg == x or pkg.startswith(x + os.sep)])

# utility to compute logged in user name
def author_name():
    import getpass
    name = getpass.getuser()
    try:
        import pwd
        login = name
        name = pwd.getpwnam(login)[4]
        name = ''.join(name.split(',')) # strip commas
    except:
        #pwd failed
        pass
    return name

def read_template(tmplf):
    p = os.path.join(roslib.packages.get_pkg_dir('roscreate'), tmplf)
    f = open(p, 'r')
    try:
        t = f.read()
    finally:
        f.close()
    return t
    
