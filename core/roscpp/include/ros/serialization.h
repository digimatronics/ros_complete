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

#ifndef ROSCPP_SERIALIZATION_H
#define ROSCPP_SERIALIZATION_H

#include "types.h"
#include "common.h"
#include "message_traits.h"
#include "builtin_message_traits.h"
#include "ros/time.h"

#include <boost/array.hpp>
#include <boost/call_traits.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/mpl/and.hpp>
#include <boost/mpl/or.hpp>
#include <boost/mpl/not.hpp>

namespace ros
{
namespace serialization
{
namespace mt = message_traits;
namespace mpl = boost::mpl;

struct Buffer
{
  Buffer()
  : data(0)
  , count(0)
  {}

  Buffer(uint8_t* _data, uint32_t _count)
  : data(_data)
  , count(_count)
  {}
  uint8_t* data;
  uint32_t count;
};

template<typename T>
struct Serializer
{
  inline static Buffer write(Buffer buffer, typename boost::call_traits<T>::param_type t)
  {
    t.serialize(buffer.data, 0);

    return Buffer();
  }

  inline static Buffer read(Buffer buffer, typename boost::call_traits<T>::reference t)
  {
    t.deserialize(buffer.data);

    return Buffer();
  }

  inline static uint32_t serializedLength(const T& t)
  {
    return t.serializationLength();
  }
};

template<typename T>
inline Buffer serialize(Buffer buffer, const T& t)
{
  return Serializer<T>::write(buffer, t);
}

template<typename T>
inline Buffer deserialize(Buffer buffer, T& t)
{
  return Serializer<T>::read(buffer, t);
}

template<typename T>
inline uint32_t serializationLength(const T& t)
{
  return Serializer<T>::serializedLength(t);
}

#define ROSCPP_CREATE_SIMPLE_SERIALIZER(Type) \
  template<> struct Serializer<Type> \
  { \
    inline static Buffer write(Buffer buffer, const Type v) \
    { \
      *reinterpret_cast<Type*>(buffer.data) = v; \
      buffer.data += sizeof(v); \
      buffer.count -= sizeof(v); \
      return buffer; \
    } \
    \
    inline static Buffer read(Buffer buffer, Type& v) \
    { \
      v = *reinterpret_cast<Type*>(buffer.data); \
      buffer.data += sizeof(v); \
      buffer.count -= sizeof(v); \
      return buffer; \
    } \
    \
    inline static uint32_t serializedLength(const Type& t) \
    { \
      return sizeof(Type); \
    } \
  };

ROSCPP_CREATE_SIMPLE_SERIALIZER(uint8_t);
ROSCPP_CREATE_SIMPLE_SERIALIZER(int8_t);
ROSCPP_CREATE_SIMPLE_SERIALIZER(uint16_t);
ROSCPP_CREATE_SIMPLE_SERIALIZER(int16_t);
ROSCPP_CREATE_SIMPLE_SERIALIZER(uint32_t);
ROSCPP_CREATE_SIMPLE_SERIALIZER(int32_t);
ROSCPP_CREATE_SIMPLE_SERIALIZER(uint64_t);
ROSCPP_CREATE_SIMPLE_SERIALIZER(int64_t);
ROSCPP_CREATE_SIMPLE_SERIALIZER(float);
ROSCPP_CREATE_SIMPLE_SERIALIZER(double);

// string
template<>
struct Serializer<std::string>
{
  inline static Buffer write(Buffer buffer, const std::string& str)
  {
    size_t len = str.size();
    buffer = serialize<uint32_t>(buffer, (uint32_t)len);
    memcpy(buffer.data, str.data(), len);
    buffer.data += len;
    buffer.count -= len;
    return buffer;
  }

  inline static Buffer read(Buffer buffer, std::string& str)
  {
    uint32_t len;
    buffer = deserialize(buffer, len);
    str = std::string((char*)buffer.data, len);
    buffer.data += len;
    buffer.count -= len;
    return buffer;
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
  inline static Buffer write(Buffer buffer, const ros::Time& v)
  {
    buffer = serialize(buffer, v.sec);
    buffer = serialize(buffer, v.nsec);
    return buffer;
  }

  inline static Buffer read(Buffer buffer, ros::Time& v)
  {
    buffer = deserialize(buffer, v.sec);
    buffer = deserialize(buffer, v.nsec);
    return buffer;
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
  inline static Buffer write(Buffer buffer, const ros::Duration& v)
  {
    buffer = serialize(buffer, v.sec);
    buffer = serialize(buffer, v.nsec);
    return buffer;
  }

  inline static Buffer read(Buffer buffer, ros::Duration& v)
  {
    buffer = deserialize(buffer, v.sec);
    buffer = deserialize(buffer, v.nsec);
    return buffer;
  }

  inline static uint32_t serializedLength(const ros::Duration& v)
  {
    return 8;
  }
};

// vectors (variable-length arrays)
// variable-length array
template<typename T, template<typename T> class Allocator, class Enabled = void>
struct VariableLengthArraySerializer
{};

template<typename T, template<typename T> class Allocator>
struct VariableLengthArraySerializer<T, Allocator, typename boost::disable_if<mt::IsFixedSize<T> >::type >
{
  typedef std::vector<T, Allocator<T> > VecType;
  typedef typename VecType::iterator IteratorType;
  typedef typename VecType::const_iterator ConstIteratorType;
  inline static Buffer write(Buffer buffer, const VecType& v)
  {
    buffer = serialize<uint32_t>(buffer, (uint32_t)v.size());
    ConstIteratorType it = v.begin();
    ConstIteratorType end = v.end();
    for (; it != end; ++it)
    {
      buffer = serialize(buffer, *it);
    }

    return buffer;
  }

  inline static Buffer read(Buffer buffer, VecType& v)
  {
    uint32_t len;
    buffer = deserialize(buffer, len);
    v.resize(len);
    IteratorType it = v.begin();
    IteratorType end = v.end();
    for (; it != end; ++it)
    {
      buffer = deserialize(buffer, *it);
    }

    return buffer;
  }

  inline static uint32_t serializedLength(const VecType& v)
  {
    uint32_t size = 4; 
    ConstIteratorType it = v.begin();
    ConstIteratorType end = v.end();
    for (; it != end; ++it)
    {
      size += serializationLength(*it);
    }

    return size;
  }
};

template<typename T, template<typename T> class Allocator>
struct VariableLengthArraySerializer<T, Allocator, typename boost::enable_if<mt::IsSimple<T> >::type >
{
  typedef std::vector<T, Allocator<T> > VecType;
  typedef typename VecType::iterator IteratorType;
  typedef typename VecType::const_iterator ConstIteratorType;
  inline static Buffer write(Buffer buffer, const VecType& v)
  {
    uint32_t len = (uint32_t)v.size();
    buffer = serialize<uint32_t>(buffer, len);
    if (!v.empty())
    {
      const uint32_t data_len = len * sizeof(T);
      memcpy(buffer.data, &v.front(), data_len);
      buffer.data += data_len;
      buffer.count -= data_len;
    }

    return buffer;
  }

  inline static Buffer read(Buffer buffer, VecType& v)
  {
    uint32_t len;
    buffer = deserialize(buffer, len);
    v.resize(len);

    if (len > 0)
    {
      const uint32_t data_len = sizeof(T) * len;
      memcpy(&v.front(), buffer.data, data_len);
      buffer.data += data_len;
      buffer.count -= data_len;
    }

    return buffer;
  }

  inline static uint32_t serializedLength(const VecType& v)
  {
    return 4 + v.size() * sizeof(T);
  }
};

template<typename T, template<typename T> class Allocator>
struct VariableLengthArraySerializer<T, Allocator, typename boost::enable_if<mpl::and_<mt::IsFixedSize<T>, mpl::not_<mt::IsSimple<T> > > >::type >
{
  typedef std::vector<T, Allocator<T> > VecType;
  typedef typename VecType::iterator IteratorType;
  typedef typename VecType::const_iterator ConstIteratorType;
  inline static Buffer write(Buffer buffer, const VecType& v)
  {
    buffer = serialize<uint32_t>(buffer, (uint32_t)v.size());
    ConstIteratorType it = v.begin();
    ConstIteratorType end = v.end();
    for (; it != end; ++it)
    {
      buffer = serialize(buffer, *it);
    }

    return buffer;
  }

  inline static Buffer read(Buffer buffer, VecType& v)
  {
    uint32_t len;
    buffer = deserialize(buffer, len);
    v.resize(len);
    IteratorType it = v.begin();
    IteratorType end = v.end();
    for (; it != end; ++it)
    {
      buffer = deserialize(buffer, *it);
    }

    return buffer;
  }

  inline static uint32_t serializedLength(const VecType& v)
  {
    uint32_t size = 4;
    if (!v.empty())
    {
      size += v.size() * serializationLength(v.front());
    }

    return size;
  }
};

template<typename T, template<typename T> class Allocator >
inline Buffer serialize(Buffer buffer, const std::vector<T, Allocator<T> >& t)
{
  return VariableLengthArraySerializer<T, Allocator>::write(buffer, t);
}

template<typename T, template<typename T> class Allocator >
inline Buffer deserialize(Buffer buffer, std::vector<T, Allocator<T> >& t)
{
  return VariableLengthArraySerializer<T, Allocator>::read(buffer, t);
}

template<typename T, template<typename T> class Allocator >
inline uint32_t serializationLength(const std::vector<T, Allocator<T> >& t)
{
  return VariableLengthArraySerializer<T, Allocator>::serializedLength(t);
}

// fixed-length arrays
template<typename T, size_t N, class Enabled = void>
struct FixedLengthArraySerializer
{};

template<typename T, size_t N>
struct FixedLengthArraySerializer<T, N, typename boost::disable_if<mt::IsFixedSize<T> >::type>
{
  typedef boost::array<T, N > ArrayType;
  typedef typename ArrayType::iterator IteratorType;
  typedef typename ArrayType::const_iterator ConstIteratorType;
  inline static Buffer write(Buffer buffer, const ArrayType& v)
  {
    ConstIteratorType it = v.begin();
    ConstIteratorType end = v.end();
    for (; it != end; ++it)
    {
      buffer = serialize(buffer, *it);
    }

    return buffer;
  }

  inline static Buffer read(Buffer buffer, ArrayType& v)
  {
    IteratorType it = v.begin();
    IteratorType end = v.end();
    for (; it != end; ++it)
    {
      buffer = deserialize(buffer, *it);
    }

    return buffer;
  }

  inline static uint32_t serializedLength(const ArrayType& v)
  {
    uint32_t size = 0;
    ConstIteratorType it = v.begin();
    ConstIteratorType end = v.end();
    for (; it != end; ++it)
    {
      size += serializationLength(*it);
    }

    return size;
  }
};

template<typename T, size_t N>
struct FixedLengthArraySerializer<T, N, typename boost::enable_if<mt::IsSimple<T> >::type>
{
  typedef boost::array<T, N > ArrayType;
  typedef typename ArrayType::iterator IteratorType;
  typedef typename ArrayType::const_iterator ConstIteratorType;
  inline static Buffer write(Buffer buffer, const ArrayType& v)
  {
    const uint32_t data_len = N * sizeof(T);
    memcpy(buffer.data, &v.front(), data_len);
    buffer.data += data_len;
    buffer.count -= data_len;

    return buffer;
  }

  inline static Buffer read(Buffer buffer, ArrayType& v)
  {
    const uint32_t data_len = N * sizeof(T);
    memcpy(&v.front(), buffer.data, data_len);
    buffer.data += data_len;
    buffer.count -= data_len;

    return buffer;
  }

  inline static uint32_t serializedLength(const ArrayType& v)
  {
    return N * sizeof(T);
  }
};

template<typename T, size_t N>
struct FixedLengthArraySerializer<T, N, typename boost::enable_if<mpl::and_<mt::IsFixedSize<T>, mpl::not_<mt::IsSimple<T> > > >::type>
{
  typedef boost::array<T, N > ArrayType;
  typedef typename ArrayType::iterator IteratorType;
  typedef typename ArrayType::const_iterator ConstIteratorType;
  inline static Buffer write(Buffer buffer, const ArrayType& v)
  {
    ConstIteratorType it = v.begin();
    ConstIteratorType end = v.end();
    for (; it != end; ++it)
    {
      buffer = serialize(buffer, *it);
    }

    return buffer;
  }

  inline static Buffer read(Buffer buffer, ArrayType& v)
  {
    IteratorType it = v.begin();
    IteratorType end = v.end();
    for (; it != end; ++it)
    {
      buffer = deserialize(buffer, *it);
    }

    return buffer;
  }

  inline static uint32_t serializedLength(const ArrayType& v)
  {
    return N * serializationLength(v.front());
  }
};

template<typename T, size_t N>
inline Buffer serialize(Buffer buffer, const boost::array<T, N>& t)
{
  return FixedLengthArraySerializer<T, N>::write(buffer, t);
}

template<typename T, size_t N>
inline Buffer deserialize(Buffer buffer, boost::array<T, N>& t)
{
  return FixedLengthArraySerializer<T, N>::read(buffer, t);
}

template<typename T, size_t N>
inline uint32_t serializationLength(const boost::array<T, N>& t)
{
  return FixedLengthArraySerializer<T, N>::serializedLength(t);
}

template<typename M>
SerializedMessage serializeMessage(const M& message)
{
  SerializedMessage m;
  m.num_bytes = serializationLength(message) + 4;
  m.buf.reset(new uint8_t[m.num_bytes]);

  Buffer b(m.buf.get(), (uint32_t)m.num_bytes);
  b = serialize(b, (uint32_t)m.num_bytes - 4);
  b = serialize(b, message);

  return m;
}

template<typename M>
SerializedMessage serializeServiceResponse(bool ok, const M& message)
{
  SerializedMessage m;

  if (ok)
  {
    m.num_bytes = serializationLength(message) + 5;
    m.buf.reset(new uint8_t[m.num_bytes]);

    Buffer b(m.buf.get(), (uint32_t)m.num_bytes);
    b = serialize(b, (uint8_t)ok);
    b = serialize(b, (uint32_t)m.num_bytes - 5);
    b = serialize(b, message);
  }
  else
  {
    m.num_bytes = 5;
    m.buf.reset(new uint8_t[5]);
    Buffer b(m.buf.get(), (uint32_t)m.num_bytes);
    b = serialize(b, (uint8_t)ok);
    b = serialize(b, (uint32_t)0);
  }

  return m;
}

template<typename M>
void deserializeMessage(const SerializedMessage& m, M& message)
{
  Buffer b(m.buf.get(), m.num_bytes);
  deserialize(b, message);
}

} // namespace serialization
} // namespace ros

#endif // ROSCPP_SERIALIZATION_H
