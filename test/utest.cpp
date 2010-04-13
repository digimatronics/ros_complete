// Copyright (c) 2009, Willow Garage, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "rosbag/rosbag.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <boost/foreach.hpp>
#include <gtest/gtest.h>

TEST(Rosbag, simplewrite)
{
  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Write);
  
  std_msgs::String str;
  str.data = std::string("foo");

  std_msgs::Int32 i;
  i.data = 42;

  bag.write("chatter", ros::Time::now(), str);
  bag.write("numbers", ros::Time::now(), i);

  bag.close();

}

TEST(Rosbag, simpleread)
{

  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string("chatter"));
  topics.push_back(std::string("numbers"));

  rosbag::MessageList messages = bag.getMessageListByTopic(topics);

  BOOST_FOREACH( rosbag::MessageInstance m, messages)
  {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL)
    {
      ASSERT_EQ(s->data, std::string("foo"));
    }

    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
    if (i != NULL)
    {
      ASSERT_EQ(i->data, 42);
    }
  }

  bag.close();

}

TEST(Rosbag, viewread)
{

  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string("chatter"));
  topics.push_back(std::string("numbers"));

  rosbag::View messages = bag.getViewByTopic(topics);

  BOOST_FOREACH(const rosbag::MessageInstance m, messages)
  {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL)
    {
      ASSERT_EQ(s->data, std::string("foo"));
    }

    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
    if (i != NULL)
    {
      ASSERT_EQ(i->data, 42);
    }
  }

  bag.close();

}


// Create a bag file where messages have a sequential count, but on random topics
// And verify it gets sorted correctly when we produce our view
TEST(Rosbag, verifytimesort)
{

  rosbag::Bag outbag;
  outbag.open("time.bag", rosbag::bagmode::Write);
  
  std_msgs::Int32 imsg;
  for (int i = 0; i < 1000; i++)
  {
    imsg.data = i;
    switch (rand() % 5)
    {
    case 0:
      outbag.write("t0", ros::Time::now(), imsg);
      break;
    case 1:
      outbag.write("t1", ros::Time::now(), imsg);
      break;
    case 2:
      outbag.write("t2", ros::Time::now(), imsg);
      break;
    case 3:
      outbag.write("t2", ros::Time::now(), imsg);
      break;
    case 4:
      outbag.write("t4", ros::Time::now(), imsg);
      break;
    }
  }

  outbag.close();


  rosbag::Bag bag;
  bag.open("time.bag", rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string("t0"));
  topics.push_back(std::string("t1"));
  topics.push_back(std::string("t2"));
  topics.push_back(std::string("t3"));
  topics.push_back(std::string("t4"));

  rosbag::View messages = bag.getViewByTopic(topics);

  int i = 0;

  BOOST_FOREACH(const rosbag::MessageInstance m, messages)
  {
    std_msgs::Int32::ConstPtr imsg = m.instantiate<std_msgs::Int32>();
    if (imsg != NULL)
    {
      ASSERT_EQ(imsg->data, i++);
    }
  }

  bag.close();

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
