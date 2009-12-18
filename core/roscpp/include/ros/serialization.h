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
#include "message_traits.h"
#include <boost/call_traits.hpp>

namespace ros
{
namespace serialization
{
namespace mt = message_traits;

template<typename T>
struct Serializer
{
  inline static void write(uint8_t*& buffer, typename boost::call_traits<T>::param_type t)
  {
    t.serialize(buffer, 0);
  }

  inline static void read(uint8_t*& buffer, uint32_t& byte_count, typename boost::call_traits<T>::reference t)
  {
    t.deserialize(buffer);
  }

  inline static uint32_t serializedLength(const T& t)
  {
    return t.serializationLength();
  }
};

template<typename T>
inline void serialize(uint8_t* buffer, const T& t)
{
  Serializer<T>::write(buffer, t);
}

template<typename T>
inline void deserialize(uint8_t* buffer, uint32_t byte_count, T& t)
{
  Serializer<T>::read(buffer, byte_count, t);
}

template<typename T>
inline uint32_t serializationLength(const T& t)
{
  return Serializer<T>::serializedLength(t);
}

} // namespace serialization
} // namespace ros

#endif // ROSCPP_SERIALIZATION_H
