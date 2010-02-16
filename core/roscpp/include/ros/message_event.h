
/*
 * Copyright (C) 2010, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSCPP_MESSAGE_EVENT_H
#define ROSCPP_MESSAGE_EVENT_H

#include "ros/forwards.h"
#include "ros/time.h"

#include <boost/type_traits/is_const.hpp>
#include <boost/utility/enable_if.hpp>

namespace ros
{

/**
 * \brief Event type for subscriptions, const ros::MessageEvent<M const>& can be used in your callback instead of const boost::shared_ptr<M const>&
 *
 * Useful if you need to retrieve meta-data about the message, such as the full connection header, or the publisher's node name
 */
template<typename M>
class MessageEvent
{
public:
  MessageEvent()
  {}

  template<typename M2>
  typename boost::enable_if<boost::is_const<M2> >::type assignMessage(const boost::shared_ptr<void const>& msg)
  {
    message_ = boost::static_pointer_cast<M2>(msg);
  }

  template<typename M2>
  typename boost::disable_if<boost::is_const<M2> >::type assignMessage(const boost::shared_ptr<void const>& msg)
  {
    message_ = boost::static_pointer_cast<M2>(boost::const_pointer_cast<void>(msg));
  }

  MessageEvent(const MessageEvent<void const>& rhs)
  {
    connection_header_ = rhs.getConnectionHeaderPtr();
    receipt_time_ = rhs.getReceiptTime();

    assignMessage<M>(rhs.getMessage());
  }

  MessageEvent(const MessageEvent<void>& rhs)
  {
    connection_header_ = rhs.getConnectionHeaderPtr();
    receipt_time_ = rhs.getReceiptTime();
    message_ = boost::static_pointer_cast<M>(rhs.getMessage());
  }

  MessageEvent(const boost::shared_ptr<M>& message, const boost::shared_ptr<M_string>& connection_header, ros::Time receipt_time)
  : message_(message)
  , connection_header_(connection_header)
  , receipt_time_(receipt_time)
  {}

  /**
   * \brief Retrieve the message
   */
  const boost::shared_ptr<M>& getMessage() const { return message_; }
  /**
   * \brief Retrieve the connection header
   */
  M_string& getConnectionHeader() const { return *connection_header_; }
  const boost::shared_ptr<M_string>& getConnectionHeaderPtr() const { return connection_header_; }

  /**
   * \brief Returns the name of the node which published this message
   */
  const std::string& getPublisherName() const { return (*connection_header_)["callerid"]; }

  /**
   * \brief Returns the time at which this message was received
   */
  ros::Time getReceiptTime() const { return receipt_time_; }

  void setMessage(const boost::shared_ptr<M>& m) { message_ = m; }
  void setConnectionHeader(const boost::shared_ptr<M_string>& h) { connection_header_ = h; }
  void setReceiptTime(ros::Time t) { receipt_time_ = t; }

private:
  boost::shared_ptr<M> message_;
  boost::shared_ptr<M_string> connection_header_;
  ros::Time receipt_time_;
};

}

#endif // ROSCPP_MESSAGE_EVENT_H
