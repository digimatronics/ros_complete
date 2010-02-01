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

#include "ros/transport/transport_tcp.h"
#if defined(WIN32)
  // This has to go after winsock2.h because MS didn't put proper inclusion
  // guards and #define guards in their headers.
  #include <Ws2tcpip.h>
#endif
#include "ros/poll_set.h"
#include "ros/header.h"
#include "ros/file_log.h"

#include <ros/assert.h>

#include <sstream>

#include <boost/bind.hpp>

#if !defined(WIN32)
  #include <sys/socket.h>
  #include <netinet/tcp.h>
  #include <sys/poll.h>
  #include <arpa/inet.h>
  #include <netdb.h>
#endif

#include <fcntl.h>
#include <errno.h>

// Joy of joys, even strerror isn't the same for Windows sockets
#if defined(WIN32)
  const int ERRNO_EAGAIN = WSAEWOULDBLOCK;
  const int ERRNO_EWOULDBLOCK = WSAEWOULDBLOCK;
  // This is hideous, but for some unknown reason calling ROS_ERROR from within
  // a macro isn't working right now.
  #define STRERROR_START() do {LPVOID strerror_result = NULL;\
    FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM,\
    NULL, WSAGetLastError(), 0, (LPTSTR) &strerror_result, 0, NULL);
  #define STRERROR_END() LocalFree(strerror_result); }while(0);
#else
  #define STRERROR_START() do {char strerror_result[256];\
    strerror_r(errno, strerror_result, 256);\
  #define STRERROR_END() }while(0);
#endif

namespace ros
{

bool TransportTCP::s_use_keepalive_ = true;

TransportTCP::TransportTCP(PollSet* poll_set, int flags)
: sock_(-1)
, closed_(false)
, expecting_read_(false)
, expecting_write_(false)
, is_server_(false)
, server_port_(-1)
, poll_set_(poll_set)
, flags_(flags)
{

}

TransportTCP::~TransportTCP()
{
  ROS_ASSERT_MSG(sock_ == -1, "TransportTCP socket [%d] was never closed", sock_);
}

bool TransportTCP::setSocket(int sock)
{
  sock_ = sock;
  return initializeSocket();
}

bool TransportTCP::initializeSocket()
{
  ROS_ASSERT(sock_ != -1);

  if (!(flags_ & SYNCHRONOUS))
  {
    // make the socket non-blocking
#if defined(WIN32)
    unsigned long setting = 1;
    if (ioctlsocket (sock_, FIONBIO, &setting) == SOCKET_ERROR)
#else
    if(fcntl(sock_, F_SETFL, O_NONBLOCK) == -1)
#endif
    {
      STRERROR_START()
      ROS_ERROR("fcntl (non-blocking) to socket [%d] failed with error [%s]", sock_, strerror_result);
      STRERROR_END()

      close();
      return false;
    }
  }

  setKeepAlive(s_use_keepalive_, 60, 10, 9);

  if (is_server_)
  {
    cached_remote_host_ = "TCPServer Socket";
  }
  else
  {
    std::stringstream ss;
    ss << getClientURI() << " on socket " << sock_;
    cached_remote_host_ = ss.str();
  }

#ifdef ROSCPP_USE_TCP_NODELAY
  setNoDelay(true);
#endif

  ROS_ASSERT(poll_set_ || (flags_ & SYNCHRONOUS));
  if (poll_set_)
  {
    poll_set_->addSocket(sock_, boost::bind(&TransportTCP::socketUpdate, this, _1), shared_from_this());
  }

  if (!(flags_ & SYNCHRONOUS))
  {
    enableRead();
  }

  return true;
}

void TransportTCP::parseHeader(const Header& header)
{
  std::string nodelay;
  if (header.getValue("tcp_nodelay", nodelay) && nodelay == "1")
  {
    ROSCPP_LOG_DEBUG("Setting nodelay on socket [%d]", sock_);
    setNoDelay(true);
  }
}

void TransportTCP::setNoDelay(bool nodelay)
{
  int flag = nodelay ? 1 : 0;
  int result = setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
  if (result < 0)
  {
    ROS_ERROR("setsockopt failed to set TCP_NODELAY on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
  }
}

void TransportTCP::setKeepAlive(bool use, uint32_t idle, uint32_t interval, uint32_t count)
{
  if (use)
  {
    ROSCPP_LOG_DEBUG("Enabling TCP Keepalive on socket [%d]", sock_);

    int val = 1;
    if (setsockopt(sock_, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&val), sizeof(val)) != 0)
    {
      ROS_ERROR("setsockopt failed to set SO_KEEPALIVE on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }

/* cygwin SOL_TCP does not seem to support TCP_KEEPIDLE, TCP_KEEPINTVL, TCP_KEEPCNT */
#if defined(SOL_TCP) && !defined(__CYGWIN__)
    val = idle;
    if (setsockopt(sock_, SOL_TCP, TCP_KEEPIDLE, &val, sizeof(val)) != 0)
    {
      ROS_ERROR("setsockopt failed to set TCP_KEEPIDLE on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }

    val = interval;
    if (setsockopt(sock_, SOL_TCP, TCP_KEEPINTVL, &val, sizeof(val)) != 0)
    {
      ROS_ERROR("setsockopt failed to set TCP_KEEPINTVL on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }

    val = count;
    if (setsockopt(sock_, SOL_TCP, TCP_KEEPCNT, &val, sizeof(val)) != 0)
    {
      ROS_ERROR("setsockopt failed to set TCP_KEEPCNT on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }
#endif
  }
  else
  {
    ROSCPP_LOG_DEBUG("Disabling TCP Keepalive on socket [%d]", sock_);

    int val = 0;
    if (setsockopt(sock_, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&val), sizeof(val)) != 0)
    {
      ROS_ERROR("setsockopt failed to set SO_KEEPALIVE on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }
  }
}

bool TransportTCP::connect(const std::string& host, int port)
{
  sock_ = socket(AF_INET, SOCK_STREAM, 0);

  if (sock_ == -1)
  {
    STRERROR_START()
    ROS_ERROR("socket() failed with error [%s]", strerror_result);
    STRERROR_END()
    return false;
  }

  sockaddr_in sin;
  sin.sin_family = AF_INET;
  if (inet_addr(host.c_str()) == INADDR_NONE)
  {
    struct addrinfo* addr;
    if (getaddrinfo(host.c_str(), NULL, NULL, &addr) != 0)
    {
      close();
      ROS_ERROR("couldn't resolve publisher host [%s]", host.c_str());
      return false;
    }

    bool found = false;
    struct addrinfo* it = addr;
    for (; it; it = it->ai_next)
    {
      if (it->ai_family == AF_INET)
      {
        memcpy(&sin, it->ai_addr, it->ai_addrlen);
        sin.sin_family = it->ai_family;
        sin.sin_port = htons(port);

        found = true;
        break;
      }
    }

    freeaddrinfo(addr);

    if (!found)
    {
      ROS_ERROR("Couldn't find an AF_INET address for [%s]\n", host.c_str());
      return false;
    }

    ROSCPP_LOG_DEBUG("Resolved publisher host [%s] to [%s]", host.c_str(), inet_ntoa(sin.sin_addr));
  }
  else
  {
    sin.sin_addr.s_addr = inet_addr(host.c_str()); // already an IP addr
  }

  sin.sin_port = htons(port);

  if (::connect(sock_, (sockaddr *)&sin, sizeof(sin)))
  {
    STRERROR_START()
    ROSCPP_LOG_DEBUG("Connect to tcpros publisher [%s:%d] failed with error [%s]", host.c_str(), port, strerror_result);
    STRERROR_END()
    close();

    return false;
  }

  if (!initializeSocket())
  {
    return false;
  }

  ROSCPP_LOG_DEBUG("Connect succeeded to [%s:%d] on socket [%d]", host.c_str(), port, sock_);

  return true;
}

bool TransportTCP::listen(int port, int backlog, const AcceptCallback& accept_cb)
{
  is_server_ = true;
  accept_cb_ = accept_cb;

  sock_ = socket(AF_INET, SOCK_STREAM, 0);

  if (sock_ <= 0)
  {
    STRERROR_START()
    ROS_ERROR("socket() failed with error [%s]", strerror_result);
    STRERROR_END()
    return false;
  }

  server_address_.sin_family = AF_INET;
  server_address_.sin_port = htons(port);
  server_address_.sin_addr.s_addr = INADDR_ANY;
  if (bind(sock_, (sockaddr *)&server_address_, sizeof(server_address_)) < 0)
  {
    STRERROR_START()
    ROS_ERROR("bind() failed with error [%s]", strerror_result);
    STRERROR_END()
    return false;
  }

  ::listen(sock_, backlog);
  socklen_t len = sizeof(server_address_);
  getsockname(sock_, (sockaddr *)&server_address_, &len);
  server_port_ = ntohs(server_address_.sin_port);

  if (!initializeSocket())
  {
    return false;
  }

  return true;
}

void TransportTCP::close()
{
  Callback disconnect_cb;

  if (!closed_)
  {
    {
      boost::recursive_mutex::scoped_lock lock(close_mutex_);

      if (!closed_)
      {
        closed_ = true;

        ROSCPP_LOG_DEBUG("TCP socket [%d] closed", sock_);

        ROS_ASSERT(sock_ != -1);

        if (poll_set_)
        {
          poll_set_->delSocket(sock_);
        }

#if defined(WIN32)
        ::shutdown(sock_, SD_BOTH);
#else
        ::shutdown(sock_, SHUT_RDWR);
#endif
#if defined(WIN32)
        if (closesocket(sock_) == SOCKET_ERROR)
#else
        if (::close(sock_) < 0)
#endif
        {
          STRERROR_START()
          ROS_ERROR("Error closing socket [%d]: [%s]", sock_, strerror_result);
          STRERROR_END()
        }

        sock_ = -1;

        disconnect_cb = disconnect_cb_;

        disconnect_cb_ = Callback();
        read_cb_ = Callback();
        write_cb_ = Callback();
        accept_cb_ = AcceptCallback();
      }
    }
  }

  if (disconnect_cb)
  {
    disconnect_cb(shared_from_this());
  }
}

int32_t TransportTCP::read(uint8_t* buffer, uint32_t size)
{
  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);
    if (closed_)
    {
      ROSCPP_LOG_DEBUG("Tried to read on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT((int32_t)size > 0);

  int num_bytes = ::recv(sock_, reinterpret_cast<char*>(buffer), size, 0);
  if (num_bytes < 0)
  {
    if (errno != EAGAIN)
    {
      STRERROR_START()
      ROSCPP_LOG_DEBUG("recv() failed with error [%s]", strerror_result);
      STRERROR_END()
    }
    else
    {
      num_bytes = 0;
    }
  }
  else if (num_bytes == 0)
  {
    ROSCPP_LOG_DEBUG("Socket [%d] received 0/%d bytes, closing", sock_, size);
    close();
    return -1;
  }

  return num_bytes;
}

int32_t TransportTCP::write(uint8_t* buffer, uint32_t size)
{
  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      ROSCPP_LOG_DEBUG("Tried to write on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT((int32_t)size > 0);

  int num_bytes = ::send(sock_, reinterpret_cast<const char*>(buffer), size, 0);
  if (num_bytes < 0)
  {
    if(errno != EAGAIN)
    {
      STRERROR_START()
      ROSCPP_LOG_DEBUG("send() failed with error [%s]", strerror_result);
      STRERROR_END()

      close();
    }
    else
    {
      num_bytes = 0;
    }
  }

  return num_bytes;
}

void TransportTCP::enableRead()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (!expecting_read_)
  {
    poll_set_->addEvents(sock_, POLLIN);
    expecting_read_ = true;
  }
}

void TransportTCP::enableWrite()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (!expecting_write_)
  {
    poll_set_->addEvents(sock_, POLLOUT);
    expecting_write_ = true;
  }
}

void TransportTCP::disableWrite()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (expecting_write_)
  {
    poll_set_->delEvents(sock_, POLLOUT);
    expecting_write_ = false;
  }
}

TransportTCPPtr TransportTCP::accept()
{
  ROS_ASSERT(is_server_);

  sockaddr client_address;
  socklen_t len = sizeof(client_address);
  int new_sock = ::accept(sock_, (sockaddr *)&client_address, &len);
  if (new_sock >= 0)
  {
    ROSCPP_LOG_DEBUG("Accepted connection on socket [%d]", new_sock);

    TransportTCPPtr transport(new TransportTCP(poll_set_, flags_));
    if (!transport->setSocket(new_sock))
    {
      ROS_ERROR("Failed to set socket on transport for socket %d", new_sock);
    }

    return transport;
  }
  else
  {
    STRERROR_START()
    ROS_ERROR("accept() on socket [%d] failed with error [%s]", sock_, strerror_result);
    STRERROR_END()
  }

  return TransportTCPPtr();
}

void TransportTCP::socketUpdate(int events)
{
  boost::recursive_mutex::scoped_lock lock(close_mutex_);

  if (closed_)
  {
    return;
  }

  if((events & POLLERR) ||
     (events & POLLHUP) ||
     (events & POLLNVAL))
  {
    ROSCPP_LOG_DEBUG("Socket %d closed with (ERR|HUP|NVAL) events %d", sock_, events);
    close();
  }
  else
  {
    if ((events & POLLIN) && expecting_read_)
    {
      if (is_server_)
      {
        // Should not block here, because poll() said that it's ready
        // for reading
        TransportTCPPtr transport = accept();
        if (transport)
        {
          ROS_ASSERT(accept_cb_);
          accept_cb_(transport);
        }
      }
      else
      {
        if (read_cb_)
        {
          read_cb_(shared_from_this());
        }
      }
    }

    if ((events & POLLOUT) && expecting_write_)
    {
      if (write_cb_)
      {
        write_cb_(shared_from_this());
      }
    }
  }
}

std::string TransportTCP::getTransportInfo()
{
  return "TCPROS connection to [" + cached_remote_host_ + "]";
}

std::string TransportTCP::getClientURI()
{
  ROS_ASSERT(!is_server_);

  sockaddr_in addr;
  socklen_t len = sizeof(addr);
  getpeername(sock_, (sockaddr *)&addr, &len);
  int port = ntohs(addr.sin_port);
  std::string ip = inet_ntoa(addr.sin_addr);

  std::stringstream uri;
  uri << ip << ":" << port;

  return uri.str();
}

} // namespace ros
