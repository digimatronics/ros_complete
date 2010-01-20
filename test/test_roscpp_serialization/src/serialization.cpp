/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

/* Author: Josh Faust */

/*
 * Test serialization templates
 */

#include <gtest/gtest.h>
#include "ros/serialization.h"
#include <boost/shared_array.hpp>

using namespace ros;
using namespace ros::serialization;

typedef boost::shared_array<uint8_t> Array;

template<typename T>
Array serializeAndDeserialize(const T& ser_val, T& deser_val)
{
  uint32_t len = serializationLength(ser_val);
  boost::shared_array<uint8_t> b(new uint8_t[len]);
  serialize(Buffer(b.get(), len), ser_val);
  deserialize(Buffer(b.get(), len), deser_val);

  return b;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Tests for compilation/validity of serialization/deserialization of primitive types
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PRIMITIVE_SERIALIZATION_TEST(Type, SerInit, DeserInit) \
  TEST(Serialization, Type) \
  { \
    Type ser_val SerInit; \
    Type deser_val DeserInit; \
    Array b = serializeAndDeserialize(ser_val, deser_val); \
    EXPECT_EQ(*(Type*)b.get(), ser_val); \
    EXPECT_EQ(ser_val, deser_val); \
  }

PRIMITIVE_SERIALIZATION_TEST(uint8_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(int8_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(uint16_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(int16_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(uint32_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(int32_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(uint64_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(int64_t, (5), (0));
PRIMITIVE_SERIALIZATION_TEST(float, (5.0f), (0.0f));
PRIMITIVE_SERIALIZATION_TEST(double, (5.0), (0.0));
PRIMITIVE_SERIALIZATION_TEST(Time, (500, 10000), (0, 0));
PRIMITIVE_SERIALIZATION_TEST(Duration, (500, 10000), (0, 0));

#define PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(Type, Start, Increment) \
  TEST(Serialization, variableLengthArray_##Type) \
  { \
    std::vector<Type> ser_val, deser_val; \
    Type val = Start; \
    for (uint32_t i = 0; i < 8; ++i) \
    { \
      ser_val.push_back(val); \
      val = val + Increment; \
    } \
    \
    Array b = serializeAndDeserialize(ser_val, deser_val); \
    EXPECT_TRUE(ser_val == deser_val); \
    \
    EXPECT_EQ(*(uint32_t*)b.get(), (uint32_t)ser_val.size()); \
    for(size_t i = 0; i < ser_val.size(); ++i) \
    { \
      Type* ptr = ((Type*)(b.get() + 4)) + i; \
      EXPECT_EQ(*ptr, ser_val[i]); \
    } \
  }

PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(uint8_t, 65, 1);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(int8_t, 65, 1);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(uint16_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(int16_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(uint32_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(int32_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(uint64_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(int64_t, 0, 100);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(float, 0.0f, 100.0f);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(double, 0.0, 100.0);
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(Time, Time(), Duration(100));
PRIMITIVE_VARIABLE_LENGTH_ARRAY_TEST(Duration, Duration(), Duration(100));

#define PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(Type, Start, Increment) \
  TEST(Serialization, fixedLengthArray_##Type) \
  { \
    boost::array<Type, 8> ser_val, deser_val; \
    Type val = Start; \
    for (uint32_t i = 0; i < 8; ++i) \
    { \
      ser_val[i] = val; \
      val = val + Increment; \
    } \
    \
    Array b = serializeAndDeserialize(ser_val, deser_val); \
    EXPECT_TRUE(ser_val == deser_val); \
    \
    for(size_t i = 0; i < ser_val.size(); ++i) \
    { \
      Type* ptr = ((Type*)b.get()) + i; \
      EXPECT_EQ(*ptr, ser_val[i]); \
    } \
  }

PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(uint8_t, 65, 1);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(int8_t, 65, 1);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(uint16_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(int16_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(uint32_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(int32_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(uint64_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(int64_t, 0, 100);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(float, 0.0f, 100.0f);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(double, 0.0, 100.0);
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(Time, Time(), Duration(100));
PRIMITIVE_FIXED_LENGTH_ARRAY_TEST(Duration, Duration(), Duration(100));

TEST(Serialization, string)
{
  std::string ser_val = "hello world";
  std::string deser_val;
  Array b = serializeAndDeserialize(ser_val, deser_val);
  EXPECT_STREQ(ser_val.c_str(), deser_val.c_str());

  EXPECT_EQ(*(uint32_t*)b.get(), (uint32_t)ser_val.size());
  EXPECT_EQ(memcmp(b.get() + 4, ser_val.data(), ser_val.size()), 0);
}

TEST(Serialization, variableLengthArray_string)
{
  std::vector<std::string> ser_val, deser_val;
  ser_val.push_back("hello world");
  ser_val.push_back("hello world22");
  ser_val.push_back("hello world333");
  ser_val.push_back("hello world4444");
  ser_val.push_back("hello world55555");
  Array b = serializeAndDeserialize(ser_val, deser_val);
  EXPECT_TRUE(ser_val == deser_val);
}

TEST(Serialization, fixedLengthArray_string)
{
  boost::array<std::string, 5> ser_val, deser_val;
  ser_val[0] = "hello world";
  ser_val[1] = "hello world22";
  ser_val[2] = "hello world333";
  ser_val[3] = "hello world4444";
  ser_val[4] = "hello world55555";
  Array b = serializeAndDeserialize(ser_val, deser_val);
  EXPECT_TRUE(ser_val == deser_val);
}



int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



