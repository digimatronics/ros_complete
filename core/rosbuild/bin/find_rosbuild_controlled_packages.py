#!/usr/bin/env python

# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
usage: %prog [options] <stack>
Find packages in a given stack that are controlled by rosbuild.
"""

import os, sys, subprocess
from optparse import OptionParser

def parse_args(argv):
  parser = OptionParser(__doc__.strip())
  parser.add_option("-r", action="store_true",
                    dest="reverse", default=False,
                    help="reverse the filter (return packages that are NOT rosbuild-controlled")
  (options, args) = parser.parse_args(argv)

  if len(args) != 2:
    parser.error("Must provide a stack")

  stack = args[1]

  return stack, options

def rosstack_contents(stack):
  cmd = ['rosstack', 'contents', stack]
  result = subprocess.Popen(cmd, stdout=subprocess.PIPE).communicate()[0]
  contents = {}
  for package in result.split():
    cmd = ['rospack', 'find', package]
    path = subprocess.Popen(cmd, stdout=subprocess.PIPE).communicate()[0]
    contents[package] = path.strip()
  return contents

def filter_contents(contents, reverse):
  packages = []
  for package in contents:
    # Look for <package_path>/CMakeLists.txt
    cmakelists = os.path.join(contents[package], 'CMakeLists.txt')
    have_cmakelists = os.path.isfile(cmakelists)
    if reverse:
      have_cmakelists = not have_cmakelists
    if have_cmakelists:
      packages.append(package)
  return packages

def main(argv, stdout):
  stack, options = parse_args(argv)

  contents = rosstack_contents(stack)
  packages = filter_contents(contents, options.reverse)
  print >> stdout, ' '.join(packages)

if __name__ == "__main__":
  main(sys.argv, sys.stdout)

