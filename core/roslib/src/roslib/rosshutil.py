'''
ROS utilities for performing tasks that would normally be performed by bash.

'''

import fnmatch
import os
import os.path
import shutil
import subprocess
import sys
import traceback


'''Functions for doing systemy stuff.'''


def curdir():
    '''This is just an alias for os.getcwd.

    It is here to save dostuff scripts from having to import os themselves.

    '''
    return os.getcwd()


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


def envvar_is_set(var, msg_on_fail=None, exit_on_fail=0):
    '''Test if an environment variable is set.

    If the variable is set, True will be returned.

    If the variable is not set and msg_on_fail is not None, that message will
    be printed.

    If the variable is not set and exit_on_fail is not 0, sys.exit will be
    called with the value of exit_on_fail as the return code.

    '''
    if var in os.environ:
        return True
    if msg_on_fail:
        print >>sys.stderr, msg_on_fail
    if exit_on_fail != 0:
        sys.exit(exit_on_fail)
    return False


def eval(cmd, working_dir=None, extra_env=None):
    '''Like run_process, but captures and returns the stdout of cmd.

    If the subprocess fails, an empty string is returned.

    If working_dir is given, the program will be run in that directory.

    If extra_env is given, it should be a dictionary with strings as keys and
    values. It is extra environment variables that should be set for the
    environment of the program. These variables will be added to the
    environment of the calling process, overwriting existing values.

    '''
    cmd = os.path.expandvars(cmd)

    new_env = os.environ
    if extra_env:
        for var in extra_env:
            new_env[var] = extra_env[var]

    if working_dir:
        working_dir = os.path.expandvars(cmd)

    p = subprocess.popen(cmd, shell=True, stdout=PIPE, cwd=working_dir,
                         env=new_env)
    stdout, stderr = p.communicate()
    if p.returncode != 0:
        return ''
    return stdout


def find(root=None, min_depth=0, include_names=None, exclude_names=None):
    '''Search for paths.

    This is a simple version of the Unix find command. It will look for files
    in the given root (or the current working directory) and below that match
    or don't match the search include and exclude terms, respectively.

    min_depth specifies the minimum level below the root of the search to
    return results from. The root is level 0. For example, a min_depth of 1
    will prevent the root appearing in the result if it matches the search
    terms.

    If no search terms are given, all files in the root that are at min_depth
    or greater will be returned.

    include_names and exclude_names should be lists of Unix-shell-style
    filenames. They may include wildcards.

    '''
    def matches_include(path):
        if not include_names:
            return True
        for pattern in include_names:
            if fnmatch.fnmatch(path, pattern):
                return True
        return False

    def matches_exclude(path):
        if not exclude_names:
            return False
        for pattern in exclude_names:
            if fnmatch.fnmatch(path, pattern):
                return True
        return False

    def in_search_depth(p, root, top_length):
        p_length = len(os.path.splitdrive(os.path.join(root, p))[1].split(os.sep))
        return (p_length - top_length) >= min_depth

    if not root:
        search_root = '.'
    else:
        search_root = os.path.expandvars(root)

    if type(include_names) is str:
        include_names = [include_names]
    if type(exclude_names) is str:
        exclude_names = [exclude_names]

    # When the search root is a file, that is the only possible result.
    if os.path.isfile(search_root):
        if min_depth > 0:
            return []
        if matches_include(search_root) and not matches_exclude(search_root):
            return [search_root]
        return []

    top_length = len(os.path.splitdrive(search_root)[1].split(os.sep))
    result=[]
    if (matches_include(search_root) and \
            not matches_exclude(search_root)) and \
            min_depth == 0:
        result.append(search_root)
    for root, dirs, files in os.walk(search_root):
        result += [os.path.join(root, p) for p in dirs + files \
                    if matches_include(p) and not matches_exclude(p) and \
                    in_search_depth(p, root, top_length)]
    return result


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


def mv(path, dest):
    '''Recursively move a path.'''
    shutil.move(path, dest)


def rm(path, recurse=False):
    '''Remove a path, optionally removing all its contents.

    If the path specifies a directory, then recurse must be True to remove it.

    If the path specifies a file, then the recurse option has no effect.

    '''
    if os.path.isdir(path):
        if recurse:
            shutil.rmtree(path)
        else:
            os.rmdir(path)
    else:
        os.remove(path)


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
        cmd_inst = subprocess.Popen(cmd, shell=True, cwd=working_dir, env=new_env,
                stdout=subprocess.PIPE, stderr=stderr_target)
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


def touch(path, create=True):
    '''Roughly equivalent to the POSIX "touch" command.

    @param create If True, and the path does not exist, an empty file will be
    created.

    @return 0

    '''
    if os.path.isfile(path):
        os.utime(path)
    elif os.path.isdir(path):
        if sys.platform != 'win32':
            # Windows does not implement directories as files
            os.utime(path)
    elif not os.path.exists(path) and create:
        f = open(path, 'w')
        f.close()
    return 0


def which(exe, msg_on_fail=None, exit_on_fail=0):
    '''Test if an executable is actually executable and return its full path.

    This tests if the path given in exe is an executable file - that is, it
    exists and the executable bit is set. If exe is not a direct reference to a
    file, then the directories in PATH will be searched.

    If exe can be executed, the absolute path to it will be returned.

    If exe cannot be found and msg_on_fail is not None, that message will be
    printed.

    If exe cannot be found and exit_on_fail is not 0, sys.exit will be called
    with the value of exit_on_fail as the return code. If exit_on_fail is 0,
    None will be returned.

    '''
    def is_executable(path):
        if os.path.exists(path) and os.access(path, os.X_OK):
            return True
        if sys.platform == 'win32' and not os.path.splitext(path)[1]:
            # If no extension, also check for .exe, .com and .bat files
            for ext in ['.exe', '.com', '.bat']:
                if os.path.exists(path + ext) and \
                        os.access(path + ext, os.X_OK):
                    return True
        return False

    def do_fail(msg_on_fail, exit_on_fail):
        if msg_on_fail:
            print >>sys.stderr, msg_on_fail
        if exit_on_fail:
            sys.exit(exit_on_fail)

    exe = os.path.expandvars(exe)
    # On the off-chance that this is a direct reference to the executable, try
    # it directly first before searching PATH.
    if is_executable(exe):
        return os.path.abspath(exe)
    # Not a direct reference, so ensure that there is no leading path specified
    if os.path.split(exe)[0]:
        # If there is, then we can't search path (try ../bin/ls in a terminal
        # some time).
        do_fail(msg_on_fail, exit_on_fail)
        return None
    # Search the directories specified in PATH
    if not os.environ['PATH']:
        # No PATH set, so fail
        do_fail(msg_on_fail, exit_on_fail)
        return None
    for path in os.environ['PATH'].split(os.pathsep):
        if is_executable(os.path.join(path, exe)):
            return os.path.abspath(exe)
    do_fail(msg_on_fail, exit_on_fail)
    return None


##############################################################################
## Variables that change between platforms.
## Each of these can be specified in the cmd argument to run_process as the
## variable specified in their leading comment.
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

