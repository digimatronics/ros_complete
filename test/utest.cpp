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
#include <boost/assign/list_of.hpp>
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

  rosbag::View view;
  view.addQuery(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(const rosbag::MessageInstance m, view)
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


TEST(Rosbag, timequery)
{

  rosbag::Bag outbag;
  outbag.open("timequery.bag", rosbag::bagmode::Write);
  
  std_msgs::Int32 imsg;

  for (int i = 0; i < 1000; i++)
  {
    imsg.data = i;
    switch (rand() % 5)
    {
    case 0:
      outbag.write("t0", ros::Time(i,0), imsg);
      break;
    case 1:
      outbag.write("t1", ros::Time(i,0), imsg);
      break;
    case 2:
      outbag.write("t2", ros::Time(i,0), imsg);
      break;
    case 3:
      outbag.write("t2", ros::Time(i,0), imsg);
      break;
    case 4:
      outbag.write("t4", ros::Time(i,0), imsg);
      break;
    }
  }
  outbag.close();


  rosbag::Bag bag;
  bag.open("timequery.bag", rosbag::bagmode::Read);
  
  rosbag::View view;
  view.addQuery(bag, rosbag::Query(ros::Time(23,0), ros::Time(782,0)));

  int i = 23;

  BOOST_FOREACH(const rosbag::MessageInstance m, view)
  {
    std_msgs::Int32::ConstPtr imsg = m.instantiate<std_msgs::Int32>();
    if (imsg != NULL)
    {
      ASSERT_EQ(imsg->data, i++);

      ASSERT_TRUE(m.getTime() < ros::Time(783,0));
    }
  }

  bag.close();
}

TEST(Rosbag, topicquery)
{

  rosbag::Bag outbag;
  outbag.open("topicquery.bag", rosbag::bagmode::Write);
  
  std_msgs::Int32 imsg;

  int j0 = 0;
  int j1 = 0;

  for (int i = 0; i < 1000; i++)
  {
    switch (rand() % 5)
    {
    case 0:
      imsg.data = j0++;
      outbag.write("t0", ros::Time(i,0), imsg);
      break;
    case 1:
      imsg.data = j0++;
      outbag.write("t1", ros::Time(i,0), imsg);
      break;
    case 2:
      imsg.data = j1++;
      outbag.write("t2", ros::Time(i,0), imsg);
      break;
    case 3:
      imsg.data = j1++;
      outbag.write("t3", ros::Time(i,0), imsg);
      break;
    case 4:
      imsg.data = j1++;
      outbag.write("t4", ros::Time(i,0), imsg);
      break;
    }
  }
  outbag.close();


  rosbag::Bag bag;
  bag.open("topicquery.bag", rosbag::bagmode::Read);
  
  std::vector<std::string> t = boost::assign::list_of("t0")("t1");

  rosbag::View view;
  view.addQuery(bag, rosbag::TopicQuery(t));

  int i = 0;

  BOOST_FOREACH(const rosbag::MessageInstance m, view)
  {
    std_msgs::Int32::ConstPtr imsg = m.instantiate<std_msgs::Int32>();
    if (imsg != NULL)
    {
      ASSERT_EQ(imsg->data, i++);
    }
  }

  bag.close();
}




TEST(Rosbag, verifymultibag)
{

  rosbag::Bag outbag1;
  outbag1.open("bag1.bag", rosbag::bagmode::Write);

  rosbag::Bag outbag2;
  outbag2.open("bag2.bag", rosbag::bagmode::Write);
  
  std_msgs::Int32 imsg;
  for (int i = 0; i < 1000; i++)
  {
    imsg.data = i;
    switch (rand() % 5)
    {
    case 0:
      outbag1.write("t0", ros::Time::now(), imsg);
      break;
    case 1:
      outbag1.write("t1", ros::Time::now(), imsg);
      break;
    case 2:
      outbag1.write("t2", ros::Time::now(), imsg);
      break;
    case 3:
      outbag2.write("t0", ros::Time::now(), imsg);
      break;
    case 4:
      outbag2.write("t1", ros::Time::now(), imsg);
      break;
    }
  }

  outbag1.close();
  outbag2.close();


  rosbag::Bag bag1;
  bag1.open("bag1.bag", rosbag::bagmode::Read);

  rosbag::Bag bag2;
  bag2.open("bag2.bag", rosbag::bagmode::Read);
  
  rosbag::View view;
  view.addQuery(bag1, rosbag::Query());
  view.addQuery(bag2, rosbag::Query());

  int i = 0;

  BOOST_FOREACH(const rosbag::MessageInstance m, view)
  {
    std_msgs::Int32::ConstPtr imsg = m.instantiate<std_msgs::Int32>();
    if (imsg != NULL)
    {
      ASSERT_EQ(imsg->data, i++);
    }
  }

  bag1.close();
  bag2.close();

}

TEST(Rosbag, modify)
{

  rosbag::Bag outbag;
  outbag.open("modify.bag", rosbag::bagmode::Write);
  
  std_msgs::Int32 imsg;

  int j0 = 0;
  int j1 = 1;

  // Create a bag with 2 interlaced topics
  for (int i = 0; i < 100; i++)
  {
    imsg.data = j0;
    j0+=2;
    outbag.write("t0", ros::Time(2*i,0), imsg);

    imsg.data = j1;
    j1+=2;
    outbag.write("t1", ros::Time(2*i+1,0), imsg);
  }
  outbag.close();


  rosbag::Bag bag;
  bag.open("modify.bag", rosbag::bagmode::Read);

  std::vector<std::string> t0 = boost::assign::list_of("t0");
  std::vector<std::string> t1 = boost::assign::list_of("t1");  

  // We're going to skip the t1 for the first half
  j0 = 0;
  j1 = 101;

  rosbag::View view;
  view.addQuery(bag, rosbag::TopicQuery(t0));

  rosbag::View::iterator iter = view.begin();
  
  for (int i = 0; i < 50; i++)
  {
    std_msgs::Int32::ConstPtr imsg = iter->instantiate<std_msgs::Int32>();

    if (imsg != NULL)
    {
      ASSERT_EQ(imsg->data, j0);
      j0+=2;
    }
    iter++;
  }

  // We now add our query, and expect it to show up
  view.addQuery(bag, rosbag::TopicQuery(t1));

  for (int i = 0; i < 50; i++)
  {
    std_msgs::Int32::ConstPtr imsg = iter->instantiate<std_msgs::Int32>();

    if (imsg != NULL)
    {
      ASSERT_EQ(imsg->data, j0);
      j0+=2;
    }

    iter++;
    imsg = iter->instantiate<std_msgs::Int32>();

    if (imsg != NULL)
    {
      ASSERT_EQ(imsg->data, j1);
      j1+=2;
    }
    iter++;
  }

  bag.close();
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
