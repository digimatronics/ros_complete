/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/poll_set.h"
#include "ros/file_log.h"

#include "ros/transport/transport.h"

#include <ros/assert.h>

#include <boost/bind.hpp>

#if defined(WIN32)
  #include <io.h>
  #include <fcntl.h>
  #define pipe(a) _pipe((a), 256, _O_BINARY)
  #define close _close
  #define write _write
  #define read _read
#else
  #include <sys/poll.h>
  #include <arpa/inet.h>
  #include <netdb.h>
#endif
#include <fcntl.h>

namespace ros
{

#if defined(WIN32)
// All code from here until the end of this #ifdef came from Player's poll
// replacement, which in turn came, via Brian, from glibc. As glibc is licensed
// under the GPL, this is almost certainly a licensing problem. A better
// solution is to replace the call to poll() above with whatever boost has to
// replace it (there's got to be something in there).

/* Event types that can be polled for.  These bits may be set in `events'
   to indicate the interesting event types; they will appear in `revents'
   to indicate the status of the file descriptor.  */
#define POLLIN          01              /* There is data to read.  */
#define POLLPRI         02              /* There is urgent data to read.  */
#define POLLOUT         04              /* Writing now will not block.  */

/* Some aliases.  */
#define POLLWRNORM      POLLOUT
#define POLLRDNORM      POLLIN
#define POLLRDBAND      POLLPRI

/* Event types always implicitly polled for.  These bits need not be set in
   `events', but they will appear in `revents' to indicate the status of
   the file descriptor.  */
#define POLLERR         010             /* Error condition.  */
#define POLLHUP         020             /* Hung up.  */
#define POLLNVAL        040             /* Invalid polling request.  */

/* Canonical number of polling requests to read in at a time in poll.  */
#define NPOLLFILE       30

/* Data structure describing a polling request.  */
struct pollfd
  {
    int fd;			/* File descriptor to poll.  */
    short int events;		/* Types of events poller cares about.  */
    short int revents;		/* Types of events that actually occurred.  */
  };


#include <sys/types.h>
#include <errno.h>
#include <string.h>
//#include <winsock2.h> // For struct timeval
#include <malloc.h> // For alloca()
#include <string.h> // For memset ()

/* *-*-nto-qnx doesn't define this constant in the system headers */
#ifndef NFDBITS
#define	NFDBITS (8 * sizeof(unsigned long))
#endif

/* Macros for counting and rounding.  */
#ifndef howmany
#define howmany(x, y)  (((x) + ((y) - 1)) / (y))
#endif
#ifndef powerof2
#define powerof2(x)     ((((x) - 1) & (x)) == 0)
#endif
#ifndef roundup
#define roundup(x, y)  ((((x) + ((y) - 1)) / (y)) * (y))
#endif

/* Poll the file descriptors described by the NFDS structures starting at
   FDS.  If TIMEOUT is nonzero and not -1, allow TIMEOUT milliseconds for
   an event to occur; if TIMEOUT is -1, block until an event occurs.
   Returns the number of file descriptors with events, zero if timed out,
   or -1 for errors.  */

int
poll(struct pollfd* fds, unsigned long int nfds, int timeout)
{
  static int max_fd_size;
  struct timeval tv;
  fd_set *rset, *wset, *xset;
  struct pollfd *f;
  int ready;
  int maxfd = 0;
  int bytes;

  if (!max_fd_size)
    max_fd_size = 256; // Best value I could find, the help doesn't specify anywhere

  bytes = howmany (max_fd_size, NFDBITS);
  rset = reinterpret_cast<fd_set*> (alloca (bytes));
  wset = reinterpret_cast<fd_set*> (alloca (bytes));
  xset = reinterpret_cast<fd_set*> (alloca (bytes));

  /* We can't call FD_ZERO, since FD_ZERO only works with sets
     of exactly FD_SETSIZE size.  */
  memset (rset, 0, bytes);
  memset (wset, 0, bytes);
  memset (xset, 0, bytes);

  for (f = fds; f < &fds[nfds]; ++f)
    {
      f->revents = 0;
      if (f->fd >= 0)
    {
      if (f->fd >= max_fd_size)
        {
          /* The user provides a file descriptor number which is higher
         than the maximum we got from the `getdtablesize' call.
         Maybe this is ok so enlarge the arrays.  */
          fd_set *nrset, *nwset, *nxset;
          int nbytes;

          max_fd_size = roundup (f->fd, NFDBITS);
          nbytes = howmany (max_fd_size, NFDBITS);

          nrset = reinterpret_cast<fd_set*> (alloca (nbytes));
          nwset = reinterpret_cast<fd_set*> (alloca (nbytes));
          nxset = reinterpret_cast<fd_set*> (alloca (nbytes));

          memset ((char *) nrset + bytes, 0, nbytes - bytes);
          memset ((char *) nwset + bytes, 0, nbytes - bytes);
          memset ((char *) nxset + bytes, 0, nbytes - bytes);

          rset = reinterpret_cast<fd_set*> (memcpy (nrset, rset, bytes));
          wset = reinterpret_cast<fd_set*> (memcpy (nwset, wset, bytes));
          xset = reinterpret_cast<fd_set*> (memcpy (nxset, xset, bytes));

          bytes = nbytes;
        }

      if (f->events & POLLIN)
        FD_SET (f->fd, rset);
      if (f->events & POLLOUT)
        FD_SET (f->fd, wset);
      if (f->events & POLLPRI)
        FD_SET (f->fd, xset);
      if (f->fd > maxfd && (f->events & (POLLIN|POLLOUT|POLLPRI)))
        maxfd = f->fd;
    }
    }

  tv.tv_sec = timeout / 1000;
  tv.tv_usec = (timeout % 1000) * 1000;

  while (1)
    {
      ready = select (maxfd + 1, rset, wset, xset,
            timeout == -1 ? NULL : &tv);

      /* It might be that one or more of the file descriptors is invalid.
     We now try to find and mark them and then try again.  */
      if (ready == -1 && errno == EBADF)
    {
      fd_set *sngl_rset = reinterpret_cast<fd_set*> (alloca (bytes));
      fd_set *sngl_wset = reinterpret_cast<fd_set*> (alloca (bytes));
      fd_set *sngl_xset = reinterpret_cast<fd_set*> (alloca (bytes));
      struct timeval sngl_tv;

      /* Clear the original set.  */
      memset (rset, 0, bytes);
      memset (wset, 0, bytes);
      memset (xset, 0, bytes);

      /* This means we don't wait for input.  */
      sngl_tv.tv_sec = 0;
      sngl_tv.tv_usec = 0;

      maxfd = -1;

      /* Reset the return value.  */
      ready = 0;

      for (f = fds; f < &fds[nfds]; ++f)
        if (f->fd != -1 && (f->events & (POLLIN|POLLOUT|POLLPRI))
        && (f->revents & POLLNVAL) == 0)
          {
        int n;

        memset (sngl_rset, 0, bytes);
        memset (sngl_wset, 0, bytes);
        memset (sngl_xset, 0, bytes);

        if (f->events & POLLIN)
          FD_SET (f->fd, sngl_rset);
        if (f->events & POLLOUT)
          FD_SET (f->fd, sngl_wset);
        if (f->events & POLLPRI)
          FD_SET (f->fd, sngl_xset);

        n = select (f->fd + 1, sngl_rset, sngl_wset, sngl_xset,
                  &sngl_tv);
        if (n != -1)
          {
            /* This descriptor is ok.  */
            if (f->events & POLLIN)
              FD_SET (f->fd, rset);
            if (f->events & POLLOUT)
              FD_SET (f->fd, wset);
            if (f->events & POLLPRI)
              FD_SET (f->fd, xset);
            if (f->fd > maxfd)
              maxfd = f->fd;
            if (n > 0)
              /* Count it as being available.  */
              ++ready;
          }
        else if (errno == EBADF)
          f->revents |= POLLNVAL;
          }
      /* Try again.  */
      continue;
    }

      break;
    }

  if (ready > 0)
    for (f = fds; f < &fds[nfds]; ++f)
      {
      if (f->fd >= 0)
        {
          if (FD_ISSET (f->fd, rset))
            f->revents |= POLLIN;
          if (FD_ISSET (f->fd, wset))
            f->revents |= POLLOUT;
          if (FD_ISSET (f->fd, xset))
            f->revents |= POLLPRI;
        }
      }

  return ready;
}

#endif // defined(WIN32)


PollSet::PollSet()
: sockets_changed_(false)
{
  signal_pipe_[0] = -1;
  signal_pipe_[1] = -1;
  // Also create a local pipe that will be used to kick us out of the
  // poll() call
  if(pipe(signal_pipe_) != 0)
  {
    ROS_FATAL( "pipe() failed");
    ROS_BREAK();
  }
#if !defined(WIN32)
  // Windows pipes can't be made non-blocking. This is probably a problem.
  if(fcntl(signal_pipe_[0], F_SETFL, O_NONBLOCK) == -1)
  {
    ROS_FATAL( "fcntl() failed");
    ROS_BREAK();
  }
  if(fcntl(signal_pipe_[1], F_SETFL, O_NONBLOCK) == -1)
  {
    ROS_FATAL( "fcntl() failed");
    ROS_BREAK();
  }
#endif

  addSocket(signal_pipe_[0], boost::bind(&PollSet::onLocalPipeEvents, this, _1));
  addEvents(signal_pipe_[0], POLLIN);
}

PollSet::~PollSet()
{
  ::close(signal_pipe_[0]);
  ::close(signal_pipe_[1]);
}

bool PollSet::addSocket(int fd, const SocketUpdateFunc& update_func, const TransportPtr& transport)
{
  SocketInfo info;
  info.fd_ = fd;
  info.events_ = 0;
  info.transport_ = transport;
  info.func_ = update_func;

  {
    boost::mutex::scoped_lock lock(socket_info_mutex_);

    bool b = socket_info_.insert(std::make_pair(fd, info)).second;
    if (!b)
    {
      ROSCPP_LOG_DEBUG("PollSet: Tried to add duplicate fd [%d]", fd);
      return false;
    }

    sockets_changed_ = true;
  }

  signal();

  return true;
}

bool PollSet::delSocket(int fd)
{
  if(fd < 0)
  {
    return false;
  }

  boost::mutex::scoped_lock lock(socket_info_mutex_);
  M_SocketInfo::iterator it = socket_info_.find(fd);
  if (it != socket_info_.end())
  {
    socket_info_.erase(it);

    sockets_changed_ = true;
    signal();

    return true;
  }

  ROSCPP_LOG_DEBUG("PollSet: Tried to delete fd [%d] which is not being tracked", fd);

  return false;
}


bool PollSet::addEvents(int sock, int events)
{
  boost::mutex::scoped_lock lock(socket_info_mutex_);

  M_SocketInfo::iterator it = socket_info_.find(sock);

  if (it == socket_info_.end())
  {
    ROSCPP_LOG_DEBUG("PollSet: Tried to add events [%d] to fd [%d] which does not exist in this pollset", events, sock);
    return false;
  }

  it->second.events_ |= events;

  signal();

  return true;
}

bool PollSet::delEvents(int sock, int events)
{
  boost::mutex::scoped_lock lock(socket_info_mutex_);

  M_SocketInfo::iterator it = socket_info_.find(sock);
  if (it != socket_info_.end())
  {
    it->second.events_ &= ~events;
  }
  else
  {
    ROSCPP_LOG_DEBUG("PollSet: Tried to delete events [%d] to fd [%d] which does not exist in this pollset", events, sock);
    return false;
  }

  signal();

  return true;
}

void PollSet::signal()
{
  boost::mutex::scoped_try_lock lock(signal_mutex_);

  if (lock.owns_lock())
  {
    char b = 0;
    if (write(signal_pipe_[1], &b, 1) < 0)
    {
      // do nothing... this prevents warnings on gcc 4.3
    }
  }
}


void PollSet::update(int poll_timeout)
{
  createNativePollset();

  // Poll across the sockets we're servicing
  int ret;
  size_t ufds_count = ufds_.size();
  if((ret = poll(&ufds_.front(), ufds_count, poll_timeout)) < 0)
  {
    // EINTR means that we got interrupted by a signal, and is not an
    // error.
    if(errno != EINTR)
    {
      ROS_ERROR("poll failed with error [%s]", strerror(errno));
    }
  }
  else if (ret > 0)
  {
    // We have one or more sockets to service
    for(size_t i=0; i<ufds_count; i++)
    {
      if (ufds_[i].revents == 0)
      {
        continue;
      }

      SocketUpdateFunc func;
      TransportPtr transport;
      int events = 0;
      {
        boost::mutex::scoped_lock lock(socket_info_mutex_);
        M_SocketInfo::iterator it = socket_info_.find(ufds_[i].fd);
        // the socket has been entirely deleted
        if (it == socket_info_.end())
        {
          continue;
        }

        const SocketInfo& info = it->second;

        // Store off the function and transport in case the socket is deleted from another thread
        func = info.func_;
        transport = info.transport_;
        events = info.events_;
      }

      if (func && (events & ufds_[i].revents))
      {
        func(ufds_[i].revents & events);
      }

      ufds_[i].revents = 0;
    }
  }
}

void PollSet::createNativePollset()
{
  boost::mutex::scoped_lock lock(socket_info_mutex_);

  if (!sockets_changed_)
  {
    return;
  }

  // Build the list of structures to pass to poll for the sockets we're servicing
  ufds_.resize(socket_info_.size());
  M_SocketInfo::iterator sock_it = socket_info_.begin();
  M_SocketInfo::iterator sock_end = socket_info_.end();
  for (int i = 0; sock_it != sock_end; ++sock_it, ++i)
  {
    const SocketInfo& info = sock_it->second;
    struct pollfd& pfd = ufds_[i];
    pfd.fd = info.fd_;
    pfd.events = info.events_;
    pfd.revents = 0;
  }
}

void PollSet::onLocalPipeEvents(int events)
{
  if(events & POLLIN)
  {
    char b;
    while(read(signal_pipe_[0], &b, 1) > 0)
    {
      //do nothing keep draining
    };
  }

}

}

