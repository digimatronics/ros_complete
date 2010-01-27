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

#include <typeinfo>

#include "ros/forwards.h"
#include "ros/message_traits.h"
#include "ros/builtin_message_traits.h"
#include "ros/serialization.h"
#include "ros/message.h"

#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/static_assert.hpp>

namespace ros
{

template<typename M>
inline boost::shared_ptr<M> defaultMessageCreateFunction()
{
  return boost::shared_ptr<M>(new M);
}

template<typename T>
typename boost::enable_if<boost::is_base_of<ros::Message, T> >::type assignConnectionHeader(T* t, const boost::shared_ptr<M_string>& connection_header)
{
  t->__connection_header = connection_header;
}

template<typename T>
typename boost::disable_if<boost::is_base_of<ros::Message, T> >::type assignConnectionHeader(T* t, const boost::shared_ptr<M_string>& connection_header)
{
}

struct SubscriptionMessageHelperDeserializeParams
{
  uint8_t* buffer;
  uint32_t length;
  boost::shared_ptr<M_string> connection_header;
};

struct SubscriptionMessageHelperCallParams
{
  VoidPtr message;
  boost::shared_ptr<M_string> connection_header;
};

template<typename M>
class MessageEvent
{
public:
  MessageEvent(const boost::shared_ptr<M const> message, const boost::shared_ptr<M_string>& connection_header)
  : message_(message)
  , connection_header_(connection_header)
  {}

  const boost::shared_ptr<M const>& getMessage() const { return message_; }
  M_string& getConnectionHeader() const { return *connection_header_; }
private:
  boost::shared_ptr<M const> message_;
  boost::shared_ptr<M_string> connection_header_;
};

/**
 * \brief Internal use
 *
 * The CallbackAdapter is templated on the callback parameter type (\b not the bare message type), and provides 3 things:
 *  - MessageType, which provides the bare message type, no const or reference qualifiers
 *  - CallbackType, which provides the full Boost.Function callback type
 *  - the static call() method, which calls the callback in the correct manner given the callback type
 *
 *  CallbackAdapter is then specialized to allow callbacks of any of the forms:
\verbatim
void callback(const boost::shared_ptr<M const>&);
void callback(boost::shared_ptr<M const>);
void callback(const M&);
void callback(M);
void callback(const MessageEvent<M>&);
\endverbatim
 */
template<typename M>
struct CallbackAdapter
{
  typedef typename boost::remove_reference<typename boost::remove_const<M>::type>::type MessageType;
  typedef boost::function<void(MessageType)> CallbackType;

  static void call(const CallbackType& cb, const SubscriptionMessageHelperCallParams& params)
  {
    cb(*boost::static_pointer_cast<MessageType>(params.message));
  }
};

template<typename M>
struct CallbackAdapter<const M&>
{
  typedef typename boost::remove_reference<typename boost::remove_const<M>::type>::type MessageType;
  typedef boost::function<void(M)> CallbackType;

  static void call(const CallbackType& cb, const SubscriptionMessageHelperCallParams& params)
  {
    cb(*boost::static_pointer_cast<MessageType>(params.message));
  }
};

template<typename M>
struct CallbackAdapter<const boost::shared_ptr<M const>& >
{
  typedef typename boost::remove_reference<typename boost::remove_const<M>::type>::type MessageType;
  typedef boost::function<void(const boost::shared_ptr<MessageType const>&)> CallbackType;

  static void call(const CallbackType& cb, const SubscriptionMessageHelperCallParams& params)
  {
    cb(boost::static_pointer_cast<MessageType>(params.message));
  }
};

template<typename M>
struct CallbackAdapter<boost::shared_ptr<M const> >
{
  typedef typename boost::remove_reference<typename boost::remove_const<M>::type>::type MessageType;
  typedef boost::function<void(const boost::shared_ptr<MessageType const>&)> CallbackType;

  static void call(const CallbackType& cb, const SubscriptionMessageHelperCallParams& params)
  {
    cb(boost::static_pointer_cast<MessageType>(params.message));
  }
};

template<typename M>
struct CallbackAdapter<const MessageEvent<M>& >
{
  typedef typename boost::remove_reference<typename boost::remove_const<M>::type>::type MessageType;
  typedef boost::function<void(const MessageEvent<MessageType>&)> CallbackType;

  static void call(const CallbackType& cb, const SubscriptionMessageHelperCallParams& params)
  {
    MessageEvent<MessageType> event(boost::static_pointer_cast<MessageType>(params.message), params.connection_header);
    cb(event);
  }
};

/**
 * \brief Abstract base class used by subscriptions to deal with concrete message types through a common
 * interface.  This is one part of the roscpp API that is \b not fully stable, so overloading this class
 * is not recommended.
 */
class SubscriptionMessageHelper
{
public:
  virtual ~SubscriptionMessageHelper() {}
  virtual VoidPtr deserialize(const SubscriptionMessageHelperDeserializeParams&) = 0;
  virtual void call(const SubscriptionMessageHelperCallParams& params) = 0;
  virtual const std::type_info& getTypeInfo() = 0;
};
typedef boost::shared_ptr<SubscriptionMessageHelper> SubscriptionMessageHelperPtr;

/**
 * \brief Concrete generic implementation of SubscriptionMessageHelper for any normal message type
 */
template<typename M, typename Enabled = void>
class SubscriptionMessageHelperT : public SubscriptionMessageHelper
{
public:
  typedef typename CallbackAdapter<M>::MessageType NonConstType;
  typedef typename boost::add_const<NonConstType>::type ConstType;
  typedef typename boost::shared_ptr<NonConstType> NonConstTypePtr;
  typedef typename boost::shared_ptr<ConstType> ConstTypePtr;

  typedef typename CallbackAdapter<M>::CallbackType Callback;
  typedef boost::function<NonConstTypePtr()> CreateFunction;
  SubscriptionMessageHelperT(const Callback& callback, const CreateFunction& create = defaultMessageCreateFunction<NonConstType>)
  : callback_(callback)
  , create_(create)
  {}

  void setCreateFunction(const CreateFunction& create)
  {
    create_ = create;
  }

  virtual VoidPtr deserialize(const SubscriptionMessageHelperDeserializeParams& params)
  {
    namespace ser = serialization;

    NonConstTypePtr msg = create_();
    assignConnectionHeader(msg.get(), params.connection_header);

    ser::IStream stream(params.buffer, params.length);
    ser::deserialize(stream, *msg);

    return VoidPtr(msg);
  }

  virtual void call(const SubscriptionMessageHelperCallParams& params)
  {
    CallbackAdapter<M>::call(callback_, params);
  }

  virtual const std::type_info& getTypeInfo()
  {
    return typeid(NonConstType);
  }

private:
  Callback callback_;
  CreateFunction create_;
};

}

#endif // ROSCPP_SUBSCRIPTION_MESSAGE_HELPER_H
