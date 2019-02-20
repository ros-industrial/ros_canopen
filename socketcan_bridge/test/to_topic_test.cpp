// Copyright (c) 2016-2019, Ivor Wanders, Mathias Lüdtke, AutonomouStuff
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <socketcan_bridge/socketcan_to_topic.hpp>

#include <can_msgs/msg/frame.hpp>
#include <socketcan_interface/socketcan.hpp>
#include <socketcan_interface/dummy.hpp>
#include <socketcan_bridge/topic_to_socketcan.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <list>

class msgCollector
{
  public:
    std::list<can_msgs::msg::Frame> messages;

    msgCollector() {}

    void msgCallback(const can_msgs::msg::Frame& f)
    {
      messages.push_back(f);
    }
};

std::string convertMessageToString(const can_msgs::msg::Frame &msg, bool lc=true) {
  can::Frame f;
  socketcan_bridge::convertMessageToSocketCAN(msg, f);
  return can::tostring(f, lc);
}

TEST(SocketCANToTopicTest, checkCorrectData)
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
  f.id = 0x1337;
  f.dlc = 8;
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

  // compare the received can_msgs::msg::Frame message to the sent can::Frame.
  can::Frame received;
  can_msgs::msg::Frame msg = message_collector_.messages.back();
  socketcan_bridge::convertMessageToSocketCAN(msg, received);

  EXPECT_EQ(received.id, f.id);
  EXPECT_EQ(received.dlc, f.dlc);
  EXPECT_EQ(received.is_extended, f.is_extended);
  EXPECT_EQ(received.is_rtr, f.is_rtr);
  EXPECT_EQ(received.is_error, f.is_error);
  EXPECT_EQ(received.data, f.data);
}

TEST(SocketCANToTopicTest, checkInvalidFrameHandling)
{
  // - tries to send a non-extended frame with an id larger than 11 bits.
  //   that should not be sent.
  // - verifies that sending one larger than 11 bits actually works.

  // sending a message with a dlc > 8 is not possible as the DummyInterface
  // causes a crash then.

  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  can::DummyInterfaceSharedPtr driver_ = std::make_shared<can::DummyInterface>(true);

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver_);
  to_topic_bridge.setup();  // initiate the message callbacks

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 1, &msgCollector::msgCallback, &message_collector_);

  // create a message
  can::Frame f;
  f.is_extended = false;
  f.id = (1<<11)+1;  // this is an illegal CAN packet... should not be sent.

  // send the can::Frame over the driver.
  // driver_->send(f);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(message_collector_.messages.size(), 0);

  f.is_extended = true;
  f.id = (1<<11)+1;  // now it should be alright.

  driver_->send(f);
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(message_collector_.messages.size(), 1);
}

TEST(SocketCANToTopicTest, checkCorrectCanIdFilter)
{
  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  can::DummyInterfaceSharedPtr driver_ = std::make_shared<can::DummyInterface>(true);

  //create can_id vector with id that should be passed and published to ros
  std::vector<unsigned int> pass_can_ids;
  pass_can_ids.push_back(0x1337);

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver_);
  to_topic_bridge.setup(can::tofilters(pass_can_ids));  // initiate the message callbacks

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
  f.id = 0x1337;
  f.dlc = 8;
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

  // compare the received can_msgs::msg::Frame message to the sent can::Frame.
  can::Frame received;
  can_msgs::msg::Frame msg = message_collector_.messages.back();
  socketcan_bridge::convertMessageToSocketCAN(msg, received);

  EXPECT_EQ(received.id, f.id);
  EXPECT_EQ(received.dlc, f.dlc);
  EXPECT_EQ(received.is_extended, f.is_extended);
  EXPECT_EQ(received.is_rtr, f.is_rtr);
  EXPECT_EQ(received.is_error, f.is_error);
  EXPECT_EQ(received.data, f.data);
}

TEST(SocketCANToTopicTest, checkInvalidCanIdFilter)
{
  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  can::DummyInterfaceSharedPtr driver_ = std::make_shared<can::DummyInterface>(true);

  //create can_id vector with id that should not be received on can bus
  std::vector<unsigned int> pass_can_ids;
  pass_can_ids.push_back(0x300);

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver_);
  to_topic_bridge.setup(can::tofilters(pass_can_ids));  // initiate the message callbacks

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
  f.id = 0x1337;
  f.dlc = 8;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = i;
  }

  // send the can frame to the driver
  driver_->send(f);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  EXPECT_EQ(0, message_collector_.messages.size());
}

TEST(SocketCANToTopicTest, checkMaskFilter)
{
  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  can::DummyInterfaceSharedPtr driver_ = std::make_shared<can::DummyInterface>(true);

  // setup filter
  can::FilteredFrameListener::FilterVector filters;
  filters.push_back(can::tofilter("300:ffe"));

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver_);
  to_topic_bridge.setup(filters);  // initiate the message callbacks

  // init the driver to test stateListener (not checked automatically).
  driver_->init("string_not_used", true);

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 10, &msgCollector::msgCallback, &message_collector_);

  const std::string pass1("300#1234"), nopass1("302#9999"), pass2("301#5678");

  // send the can framew to the driver
  driver_->send(can::toframe(pass1));
  driver_->send(can::toframe(nopass1));
  driver_->send(can::toframe(pass2));

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  // compare the received can_msgs::msg::Frame message to the sent can::Frame.
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
