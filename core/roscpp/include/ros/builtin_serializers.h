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

#ifndef ROSCPP_BUILTIN_SERIALIZERS_H
#define ROSCPP_BUILTIN_SERIALIZERS_H

#include "message_traits.h"
#include "ros/time.h"

namespace ros
{
namespace serialization
{
namespace mt = message_traits;

#define ROSCPP_CREATE_PRIMITIVE_SERIALIZER(Type) \
  template<> struct Serializer<Type> \
  { \
    inline static void write(uint8_t*& buffer, const Type v) \
    { \
      *reinterpret_cast<Type*>(buffer) = v; \
      buffer += sizeof(v); \
    } \
    \
    inline static void read(uint8_t*& buffer, uint32_t& byte_count, Type& v) \
    { \
      v = *reinterpret_cast<Type*>(buffer); \
      buffer += sizeof(v); \
      byte_count -= sizeof(v); \
    } \
    \
    inline static uint32_t serializedLength(const Type& t) \
    { \
      return sizeof(Type); \
    } \
  };

ROSCPP_CREATE_PRIMITIVE_SERIALIZER(uint8_t);
ROSCPP_CREATE_PRIMITIVE_SERIALIZER(int8_t);
ROSCPP_CREATE_PRIMITIVE_SERIALIZER(uint16_t);
ROSCPP_CREATE_PRIMITIVE_SERIALIZER(int16_t);
ROSCPP_CREATE_PRIMITIVE_SERIALIZER(uint32_t);
ROSCPP_CREATE_PRIMITIVE_SERIALIZER(int32_t);
ROSCPP_CREATE_PRIMITIVE_SERIALIZER(uint64_t);
ROSCPP_CREATE_PRIMITIVE_SERIALIZER(int64_t);
ROSCPP_CREATE_PRIMITIVE_SERIALIZER(float);
ROSCPP_CREATE_PRIMITIVE_SERIALIZER(double);

// string
template<>
struct Serializer<std::string>
{
  inline static void write(uint8_t*& buffer, const std::string& str)
  {
    size_t len = str.size();
    serialize<uint32_t>(buffer, (uint32_t)len);
    memcpy(buffer, str.data(), len);
    buffer += len;
  }

  inline static void read(uint8_t*& buffer, uint32_t& byte_count, std::string& str)
  {
    uint32_t len;
    deserialize(buffer, byte_count, len);
    str = std::string((char*)buffer, len);
    buffer += len;
    byte_count -= len;
  }

  inline static uint32_t serializedLength(const std::string& str)
  {
    return 4 + (uint32_t)str.size();
  }
};

// time
template<>
struct Serializer<ros::Time>
{
  inline static void write(uint8_t*& buffer, const ros::Time& v)
  {
    serialize(buffer, v.sec);
    serialize(buffer, v.nsec);
  }

  inline static void read(uint8_t*& buffer, uint32_t& byte_count, ros::Time& v)
  {
    deserialize(buffer, byte_count, v.sec);
    deserialize(buffer, byte_count, v.nsec);
  }

  inline static uint32_t serializedLength(const ros::Time& v)
  {
    return 8;
  }
};

// duration
template<>
struct Serializer<ros::Duration>
{
  inline static void write(uint8_t*& buffer, const ros::Duration& v)
  {
    serialize(buffer, v.sec);
    serialize(buffer, v.nsec);
  }

  inline static void read(uint8_t*& buffer, uint32_t& byte_count, ros::Duration& v)
  {
    deserialize(buffer, byte_count, v.sec);
    deserialize(buffer, byte_count, v.nsec);
  }

  inline static uint32_t serializedLength(const ros::Duration& v)
  {
    return 8;
  }
};


} // namespace serialization
} // namespace ros

#endif // ROSCPP_BUILTIN_SERIALIZERS_H

