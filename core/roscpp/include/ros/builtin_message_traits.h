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

#ifndef ROSCPP_BUILTIN_MESSAGE_TRAITS_H
#define ROSCPP_BUILTIN_MESSAGE_TRAITS_H

#include "message.h"
#include "roslib/Header.h"

namespace ros
{
namespace message_traits
{

#define ROSCPP_CREATE_PRIMITIVE_TRAITS(Type) \
    template<> struct IsPrimitive<Type> : public TrueType {}; \
    template<> struct IsFixedSize<Type> : public TrueType {};

ROSCPP_CREATE_PRIMITIVE_TRAITS(uint8_t);
ROSCPP_CREATE_PRIMITIVE_TRAITS(int8_t);
ROSCPP_CREATE_PRIMITIVE_TRAITS(uint16_t);
ROSCPP_CREATE_PRIMITIVE_TRAITS(int16_t);
ROSCPP_CREATE_PRIMITIVE_TRAITS(uint32_t);
ROSCPP_CREATE_PRIMITIVE_TRAITS(int32_t);
ROSCPP_CREATE_PRIMITIVE_TRAITS(uint64_t);
ROSCPP_CREATE_PRIMITIVE_TRAITS(int64_t);
ROSCPP_CREATE_PRIMITIVE_TRAITS(float);
ROSCPP_CREATE_PRIMITIVE_TRAITS(double);
ROSCPP_CREATE_PRIMITIVE_TRAITS(Time);
ROSCPP_CREATE_PRIMITIVE_TRAITS(Duration);

} // namespace message_traits
} // namespace ros

#endif // ROSCPP_BUILTIN_MESSAGE_TRAITS_H

