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
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#ifndef ROSCPP_MESSAGE_TRAITS_H
#define ROSCPP_MESSAGE_TRAITS_H

#include "message.h"

namespace roslib
{
struct Header;
}

namespace ros
{
namespace message_traits
{

struct TrueType
{
  static const bool value = true;
};

struct FalseType
{
  static const bool value = false;
};

template<typename M> struct IsSimple : public FalseType {};
template<typename M> struct IsFixedSize : public FalseType {};
template<typename M> struct HasHeader : public FalseType {};

template<typename M>
inline const char* md5sum()
{
  return M::__s_getMD5Sum().c_str();
}

template<typename M>
inline const char* datatype()
{
  return M::__s_getDataType().c_str();
}

template<typename M>
inline const char* definition()
{
  return M::__s_getMessageDefinition().c_str();
}

template<typename M>
inline roslib::Header* getHeader(M& msg)
{
  return 0;
}

template<typename M>
inline bool isSimple()
{
  return IsSimple<M>::value;
}

template<typename M>
inline bool isFixedSize()
{
  return IsFixedSize<M>::value;
}

template<typename M>
inline bool hasHeader()
{
  return HasHeader<M>::value;
}

struct Traits
{
  bool has_header;
  roslib::Header* header;
  bool is_simple;
  bool is_fixed_size;

  std::string md5sum;
  std::string datatype;
  std::string definition;
};

template<typename M>
inline Traits traits()
{
  Traits t;
  t.has_header = hasHeader<M>();
  t.header = getHeader<M>();
  t.is_simple = isSimple<M>();
  t.is_fixed_size = isFixedSize<M>();
  t.md5sum = md5sum<M>();
  t.datatype = datatype<M>();
  t.definition = definition<M>();

  return t;
}

} // namespace message_traits
} // namespace ros

#endif // ROSCPP_MESSAGE_TRAITS_H
