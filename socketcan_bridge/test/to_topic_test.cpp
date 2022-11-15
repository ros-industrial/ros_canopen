/*
 * Copyright (c) 2016, Ivor Wanders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
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
#include <socketcan_bridge/socketcan_to_topic.h>

#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/dummy.h>
#include <socketcan_bridge/topic_to_socketcan.h>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <list>
#include <memory>
#include <string>
#include <vector>

class msgCollector
{
  public:
    std::list<can_msgs::Frame> messages;

    msgCollector() {}

    void msgCallback(const can_msgs::Frame& f)
    {
      messages.push_back(f);
    }
};

std::string convertMessageToString(const can_msgs::Frame &msg, bool lc = true)
{
  can::Frame f;
  socketcan_bridge::convertMessageToSocketCAN(msg, f);
  return can::tostring(f, lc);
}

TEST(SocketCANToTopicTest, checkCorrectData)
{
  ros::NodeHandle nh(""), nh_param("~");

  can::DummyBus bus("checkCorrectData");

  // create the dummy interface
  can::ThreadedDummyInterfaceSharedPtr dummy = std::make_shared<can::ThreadedDummyInterface>();

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, dummy);
  to_topic_bridge.setup();  // initiate the message callbacks

  // init the driver to test stateListener (not checked automatically).
  dummy->init(bus.name, true, can::NoSettings::create());

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 1, &msgCollector::msgCallback, &message_collector_);

  // create a can frame
  can::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.is_error = false;
  f.id = 0x1337;
  f.dlc = 8;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = i;
  }

  // send the can frame to the driver
  dummy->send(f);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(1, message_collector_.messages.size());

  // compare the received can_msgs::Frame message to the sent can::Frame.
  can::Frame received;
  can_msgs::Frame msg = message_collector_.messages.back();
  socketcan_bridge::convertMessageToSocketCAN(msg, received);

  EXPECT_EQ(received.id, f.id);
  EXPECT_EQ(received.dlc, f.dlc);
  EXPECT_EQ(received.is_extended, f.is_extended);
  EXPECT_EQ(received.is_rtr, f.is_rtr);
  EXPECT_EQ(received.is_error, f.is_error);
  for (int i = 0; i < f.dlc; ++i)
  {
    EXPECT_EQ(received.data[i], f.data[i]);
  }
}

TEST(SocketCANToTopicTest, checkCorrectFdData)
{
  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  can::DummyInterfaceSharedPtr driver_ = std::make_shared<can::DummyInterface>(true);

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver_);
  to_topic_bridge.setup();  // initiate the message callbacks

  // init the driver to test stateListener (not checked automatically).
  driver_->init("string_not_used", true);

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 1, &msgCollector::msgCallback, &message_collector_);

  // create a can frame
  can::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.is_error = false;
  f.id = 0x123;
  f.dlc = 64;
  f.is_fd = true;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = i;
  }

  // send the can frame to the driver
  driver_->send(f);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(1, message_collector_.messages.size());

  // compare the received can_msgs::Frame message to the sent can::Frame.
  can::Frame received;
  can_msgs::Frame msg = message_collector_.messages.back();
  socketcan_bridge::convertMessageToSocketCAN(msg, received);

  EXPECT_EQ(received.id, f.id);
  EXPECT_EQ(received.dlc, f.dlc);
  EXPECT_EQ(received.is_extended, f.is_extended);
  EXPECT_EQ(received.is_rtr, f.is_rtr);
  EXPECT_EQ(received.is_error, f.is_error);
  EXPECT_EQ(received.is_fd, f.is_fd);
  for (int i = 0; i < f.dlc; ++i)
  {
    EXPECT_EQ(received.data[i], f.data[i]);
  }
}

TEST(SocketCANToTopicTest, checkInvalidFrameHandling)
{
  // - tries to send a non-extended frame with an id larger than 11 bits.
  //   that should not be sent.
  // - verifies that sending one larger than 11 bits actually works.

  // sending a message with a dlc > 64 is not possible as the DummyInterface
  // causes a crash then.

  ros::NodeHandle nh(""), nh_param("~");

  can::DummyBus bus("checkInvalidFrameHandling");

  // create the dummy interface
  can::ThreadedDummyInterfaceSharedPtr dummy = std::make_shared<can::ThreadedDummyInterface>();

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, dummy);
  to_topic_bridge.setup();  // initiate the message callbacks

  dummy->init(bus.name, true, can::NoSettings::create());

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 1, &msgCollector::msgCallback, &message_collector_);

  // create a message
  can::Frame f;
  f.is_extended = false;
  f.id = (1<<11)+1;  // this is an illegal CAN packet... should not be sent.

  // send the can::Frame over the driver.
  // dummy->send(f);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(message_collector_.messages.size(), 0);

  f.is_extended = true;
  f.id = (1<<11)+1;  // now it should be alright.

  dummy->send(f);
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(message_collector_.messages.size(), 1);
}

TEST(SocketCANToTopicTest, checkCorrectCanIdFilter)
{
  ros::NodeHandle nh(""), nh_param("~");

  can::DummyBus bus("checkCorrectCanIdFilter");

  // create the dummy interface
  can::ThreadedDummyInterfaceSharedPtr dummy = std::make_shared<can::ThreadedDummyInterface>();

  // create can_id vector with id that should be passed and published to ros
  std::vector<unsigned int> pass_can_ids;
  pass_can_ids.push_back(0x1337);

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, dummy);
  to_topic_bridge.setup(can::tofilters(pass_can_ids));  // initiate the message callbacks

  // init the driver to test stateListener (not checked automatically).
  dummy->init(bus.name, true, can::NoSettings::create());

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 1, &msgCollector::msgCallback, &message_collector_);

  // create a can frame
  can::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.is_error = false;
  f.id = 0x1337;
  f.dlc = 8;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = i;
  }

  // send the can frame to the driver
  dummy->send(f);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(1, message_collector_.messages.size());

  // compare the received can_msgs::Frame message to the sent can::Frame.
  can::Frame received;
  can_msgs::Frame msg = message_collector_.messages.back();
  socketcan_bridge::convertMessageToSocketCAN(msg, received);

  EXPECT_EQ(received.id, f.id);
  EXPECT_EQ(received.dlc, f.dlc);
  EXPECT_EQ(received.is_extended, f.is_extended);
  EXPECT_EQ(received.is_rtr, f.is_rtr);
  EXPECT_EQ(received.is_error, f.is_error);
  for (int i = 0; i < f.dlc; ++i)
  {
    EXPECT_EQ(received.data[i], f.data[i]);
  }
}

TEST(SocketCANToTopicTest, checkInvalidCanIdFilter)
{
  ros::NodeHandle nh(""), nh_param("~");

  can::DummyBus bus("checkInvalidCanIdFilter");

  // create the dummy interface
  can::ThreadedDummyInterfaceSharedPtr dummy = std::make_shared<can::ThreadedDummyInterface>();

  // create can_id vector with id that should not be received on can bus
  std::vector<unsigned int> pass_can_ids;
  pass_can_ids.push_back(0x300);

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, dummy);
  to_topic_bridge.setup(can::tofilters(pass_can_ids));  // initiate the message callbacks

  // init the driver to test stateListener (not checked automatically).
  dummy->init(bus.name, true, can::NoSettings::create());

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 1, &msgCollector::msgCallback, &message_collector_);

  // create a can frame
  can::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.is_error = false;
  f.id = 0x1337;
  f.dlc = 8;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = i;
  }

  // send the can frame to the driver
  dummy->send(f);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  EXPECT_EQ(0, message_collector_.messages.size());
}

TEST(SocketCANToTopicTest, checkMaskFilter)
{
  ros::NodeHandle nh(""), nh_param("~");

  can::DummyBus bus("checkMaskFilter");

  // create the dummy interface
  can::ThreadedDummyInterfaceSharedPtr dummy = std::make_shared<can::ThreadedDummyInterface>();

  // setup filter
  can::FilteredFrameListener::FilterVector filters;
  filters.push_back(can::tofilter("300:ffe"));

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, dummy);
  to_topic_bridge.setup(filters);  // initiate the message callbacks

  // init the driver to test stateListener (not checked automatically).
  dummy->init(bus.name, true, can::NoSettings::create());

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 10, &msgCollector::msgCallback, &message_collector_);

  const std::string pass1("300#1234"), nopass1("302#9999"), pass2("301#5678");

  // send the can framew to the driver
  dummy->send(can::toframe(pass1));
  dummy->send(can::toframe(nopass1));
  dummy->send(can::toframe(pass2));

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  // compare the received can_msgs::Frame message to the sent can::Frame.
  ASSERT_EQ(2, message_collector_.messages.size());
  EXPECT_EQ(pass1, convertMessageToString(message_collector_.messages.front()));
  EXPECT_EQ(pass2, convertMessageToString(message_collector_.messages.back()));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_to_topic");
  ros::NodeHandle nh;
  ros::WallDuration(1.0).sleep();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
