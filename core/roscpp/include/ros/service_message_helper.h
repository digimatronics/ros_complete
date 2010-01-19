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

namespace ros
{

/**
 * \brief Abstract base class used by service servers to deal with concrete message types through a common
 * interface.  This is one part of the roscpp API that is \b not fully stable, so overloading this class
 * is not recommended unless you have an explicit need (like to implement a scripting interface).
 */
class ServiceMessageHelper
{
public:
  virtual ~ServiceMessageHelper() {}
  virtual bool call(const SerializedMessage& req, SerializedMessage& res, const boost::shared_ptr<M_string>& connection_header) = 0;

  /**
   * \brief Returns the md5sum of this service
   */
  virtual std::string getMD5Sum() = 0;
  /**
   * \brief Returns the datatype of this service
   */
  virtual std::string getDataType() = 0;
  /**
   * \brief Returns the datatype of the request message
   */
  virtual std::string getRequestDataType() = 0;
  /**
   * \brief Returns the datatype of the response message
   */
  virtual std::string getResponseDataType() = 0;
};
typedef boost::shared_ptr<ServiceMessageHelper> ServiceMessageHelperPtr;

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

  ServiceMessageHelperT(const Callback& callback)
  : callback_(callback)
  {
    namespace st = service_traits;
    namespace mt = message_traits;
    md5sum_ = st::md5sum<MReq>();
    data_type_ = st::datatype<MReq>();
    req_data_type_ = mt::datatype<MReq>();
    res_data_type_ = mt::datatype<MRes>();
  }

  ServiceMessageHelperT(const Callback& callback, const std::string& md5sum, const std::string& data_type, const std::string& req_data_type, const std::string& res_data_type)
  : callback_(callback)
  , md5sum_(md5sum)
  , data_type_(data_type)
  , req_data_type_(req_data_type)
  , res_data_type_(res_data_type)
  {}

  virtual bool call(const SerializedMessage& req_bytes, SerializedMessage& res_bytes, const boost::shared_ptr<M_string>& connection_header)
  {
    namespace ser = serialization;
    MReqPtr req(new MReq);
    MResPtr res(new MRes);

    ser::deserializeMessage(req_bytes, *req);
    req->__connection_header = connection_header;

    bool ok = callback_(*req, *res);
    res_bytes = ser::serializeServiceResponse(ok, *res);
    return ok;
  }

  virtual std::string getMD5Sum() { return md5sum_; }
  virtual std::string getDataType() { return data_type_; }
  virtual std::string getRequestDataType() { return req_data_type_; }
  virtual std::string getResponseDataType() { return res_data_type_; }

private:
  Callback callback_;
  std::string md5sum_;
  std::string data_type_;
  std::string req_data_type_;
  std::string res_data_type_;
};

}

#endif // ROSCPP_SERVICE_MESSAGE_HELPER_H
