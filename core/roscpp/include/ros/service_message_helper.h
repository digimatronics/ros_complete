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

#ifndef ROSCPP_SERVICE_MESSAGE_HELPER_H
#define ROSCPP_SERVICE_MESSAGE_HELPER_H

#include "ros/forwards.h"
#include "ros/common.h"
#include "ros/message.h"
#include "ros/message_traits.h"
#include "ros/service_traits.h"
#include "ros/serialization.h"

#include <boost/type_traits/is_base_of.hpp>
#include <boost/utility/enable_if.hpp>

namespace ros
{
struct ServiceMessageHelperCallParams
{
  SerializedMessage request;
  SerializedMessage response;
  boost::shared_ptr<M_string> connection_header;
};

/**
 * \brief Abstract base class used by service servers to deal with concrete message types through a common
 * interface.  This is one part of the roscpp API that is \b not fully stable, so overloading this class
 * is not recommended unless you have an explicit need (like to implement a scripting interface).
 */
class ServiceMessageHelper
{
public:
  virtual ~ServiceMessageHelper() {}
  virtual bool call(ServiceMessageHelperCallParams& params) = 0;
};
typedef boost::shared_ptr<ServiceMessageHelper> ServiceMessageHelperPtr;

template<typename M>
inline boost::shared_ptr<M> defaultServiceCreateFunction()
{
  return boost::shared_ptr<M>(new M);
}

/**
 * \brief Concrete generic implementation of ServiceMessageHelper for any normal service type
 */
template<class MReq, class MRes>
class ServiceMessageHelperT : public ServiceMessageHelper
{
public:
  typedef boost::shared_ptr<MReq> MReqPtr;
  typedef boost::shared_ptr<MRes> MResPtr;
  typedef boost::function<bool(MReq&, MRes&)> Callback;
  typedef boost::function<MReqPtr()> ReqCreateFunction;
  typedef boost::function<MResPtr()> ResCreateFunction;

  ServiceMessageHelperT(const Callback& callback, const ReqCreateFunction& create_req = defaultServiceCreateFunction<MReq>, const ResCreateFunction& create_res = defaultServiceCreateFunction<MRes>)
  : callback_(callback)
  , create_req_(create_req)
  , create_res_(create_res)
  {
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

  virtual bool call(ServiceMessageHelperCallParams& params)
  {
    namespace ser = serialization;
    MReqPtr req(create_req_());
    MResPtr res(create_res_());

    ser::deserializeMessage(params.request, *req);
    assignConnectionHeader(req.get(), params.connection_header);

    bool ok = callback_(*req, *res);
    params.response = ser::serializeServiceResponse(ok, *res);
    return ok;
  }

private:
  Callback callback_;
  ReqCreateFunction create_req_;
  ResCreateFunction create_res_;
};

}

#endif // ROSCPP_SERVICE_MESSAGE_HELPER_H
