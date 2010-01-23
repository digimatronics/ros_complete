/*
 * Copyright (C) 2009, Willow Garage, Inc.
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

#ifndef ROSCPP_SUBSCRIPTION_MESSAGE_HELPER_H
#define ROSCPP_SUBSCRIPTION_MESSAGE_HELPER_H

#include "ros/forwards.h"
#include "ros/message_traits.h"
#include "ros/builtin_message_traits.h"
#include "ros/serialization.h"
#include "ros/message.h"

#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/utility/enable_if.hpp>

namespace ros
{

/**
 * \brief Abstract base class used by subscriptions to deal with concrete message types through a common
 * interface.  This is one part of the roscpp API that is \b not fully stable, so overloading this class
 * is not recommended unless you have an explicit need (like to implement a scripting interface).
 */
class SubscriptionMessageHelper
{
public:
  virtual ~SubscriptionMessageHelper() {}
  virtual VoidPtr deserialize(uint8_t* buffer, uint32_t length, const boost::shared_ptr<M_string>& connection_header) = 0;

  virtual void call(const VoidPtr& msg) = 0;
};
typedef boost::shared_ptr<SubscriptionMessageHelper> SubscriptionMessageHelperPtr;

/**
 * \brief Concrete generic implementation of SubscriptionMessageHelper for any normal message type
 */
template<typename M, typename Enabled = void>
class SubscriptionMessageHelperT : public SubscriptionMessageHelper
{
public:
  typedef boost::shared_ptr<M> MPtr;
  typedef boost::function<void (const MPtr&)> Callback;
  SubscriptionMessageHelperT(const Callback& callback)
  : callback_(callback)
  {}

  template<typename T>
  typename boost::enable_if<boost::is_base_of<ros::Message, T> >::type assignConnectionHeader(T* t, const boost::shared_ptr<M_string>& connection_header)
  {
    t->__connection_header = connection_header;
  }

  template<typename T>
  typename boost::disable_if<boost::is_base_of<ros::Message, T> >::type assignConnectionHeader(T* t, const boost::shared_ptr<M_string>& connection_header)
  {
  }

  virtual VoidPtr deserialize(uint8_t* buffer, uint32_t length, const boost::shared_ptr<M_string>& connection_header)
  {
    namespace ser = serialization;

    typedef typename boost::remove_const<M>::type NonConstType;
    NonConstType* msg = new NonConstType;

    ser::Stream stream(buffer, length);
    ser::deserialize(stream, *msg);

    assignConnectionHeader(msg, connection_header);

    return VoidPtr(msg);
  }

  virtual void call(const VoidPtr& msg)
  {
    MPtr casted_msg = boost::static_pointer_cast<M>(msg);
    callback_(casted_msg);
  }

private:
  Callback callback_;
};

}

#endif // ROSCPP_SUBSCRIPTION_MESSAGE_HELPER_H
