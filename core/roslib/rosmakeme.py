#!/usr/bin/env python


import os
import os.path
import subprocess
import sys


platform_vars = {}
# ${MAKE_CMD}
if sys.platform == 'win32':
    platform_vars['MAKE'] = 'nmake'
else:
    platform_vars['MAKE'] = os.environ.get('MAKE', 'make')


def subst_platform_vars(string):
    for var in platform_vars:
        string = string.replace('${%s}' % var, platform_vars[var])
    return string


def envvar(var):
    '''Get an environment variable, or an empty string if it is not set.

    The standard os.getenv() function returns None if the variable is not set.
    This makes it difficult to use directly in strings. This function will
    return an empty string if the environment variable is not set, making it
    safe to use directly in string substitutions.

    Note that many of the functions in this module (such as run_process) will
    call os.expandvars() on some of their arguments to expand environment
    variables themselves. However, if the variable is _not_ set, it will remain
    unchanged in the string, which can cause problems with some commands.

    '''
    if var in os.environ:
        return os.environ[var]
    return ''


def mkdir(path, make_parents=False, mode=None, ignore_existing=True):
    '''Make a directory, optionally making all necessary parent directories.

    If mode is not None, then the directory will be created with that mode
    (e.g. 0700 - note that this is octal). This may be ignored on some
    platforms. If mode is None, then the default of 0777 will be used.

    Note that, if make_parents is True, then the given path must not contain
    any parent directory specifiers (e.g. ..).

    '''
    path = os.path.abspath(os.path.expandvars(path))
    try:
        if make_parents:
            if mode:
                return os.makedirs(path, mode)
            else:
                return os.makedirs(path)
        else:
            if mode:
                return os.mkdir(path, mode)
            else:
                return os.mkdir(path)
    except OSError, e:
        if e.errno == 17 and ignore_existing:
            return 0
        else:
            raise 


def run_process(cmd, msg_on_fail=None, working_dir=None, extra_env=None,
                exit_on_fail=1, communicate=False, merge_output=False):
    '''Run a program in a subprocess, waiting for it to finish.

    On failure, the specified error message is printed, along with the return
    code of the process.

    @param working_dir If given, the program will be run in this directory.

    @param extra_env If given, it should be a dictionary with strings as keys
    and values. It is extra environment variables that should be set for the
    environment of the program. These variables will be added to the
    environment of the calling process, overwriting existing values.

    @param exit_on_fail If not zero, sys.exit will be called with the value of
    this as the return code.

    @param merge_output Merge stdout and stderr together into a single stream,
    returned in the stdout component of the return value. The stderr component
    of the return value will be empty.

    @param communicate If true, save the stdout and stderr locally and return
    them. Otherwise, they will go to their usual destination (most likely, the
    terminal) and the stdout and stderr components of the return value will be
    empty (in case of an error, stdout and stderr will be printed anyway).

    @return The return code, stdout and stderr of the called process.

    '''
    cmd = subst_platform_vars(os.path.expandvars(cmd))

    new_env = os.environ.copy()
    if extra_env:
        for var in extra_env:
            new_env[var] = extra_env[var]

    if working_dir:
        working_dir = os.path.expandvars(working_dir)

    if communicate:
        if merge_output:
            stderr_target=subprocess.STDOUT
        else:
            stderr_target=subprocess.PIPE
        cmd_inst = subprocess.Popen(cmd, shell=True, cwd=working_dir,
                env=new_env, stdout=subprocess.PIPE, stderr=stderr_target)
        cmd_stdout, cmd_stderr = cmd_inst.communicate()
        retcode = cmd_inst.returncode
    else:
        retcode = subprocess.call(cmd, shell=True, cwd=working_dir,
                env=new_env)
        cmd_stdout = ''
        cmd_stderr = ''

    if retcode != 0:
        if msg_on_fail:
            print >>sys.stderr, msg_on_fail + ' [Exit code: %d]' % retcode
        else:
            print >>sys.stderr, 'Error executing %s [Exit code: %d]' % \
                    (cmd, retcode)
        if cmd_stdout:
            print cmd_stdout
        if cmd_stderr:
            print cmd_stderr
        if exit_on_fail != 0:
            sys.exit(exit_on_fail)
    return retcode, cmd_stdout, cmd_stderr


if sys.platform == 'win32':
    win32_generator_flag = '-G "NMake Makefiles"'
else:
    win32_generator_flag = ''
mkdir('build', make_parents=True)
mkdir('bin', make_parents=True)
if os.path.isdir('msg/cpp'):
    os.rmdir('msg/cpp')
if os.path.isdir('srv/cpp'):
    os.rmdir('srv/cpp')
ret = run_process('cmake -Wdev %s ..' % win32_generator_flag,
        working_dir='build')[0]
if ret == 0:
    run_process('${MAKE} %s' % envvar('ROS_PARALLEL_JOBS'), working_dir='build')

