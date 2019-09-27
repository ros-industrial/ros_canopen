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

#include <gtest/gtest.h>

#include <socketcan_bridge/topic_to_socketcan.hpp>
#include <socketcan_bridge/socketcan_to_topic.hpp>
#include <socketcan_interface/socketcan.hpp>
#include <socketcan_interface/dummy.hpp>

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>

#include <list>
#include <memory>

class frameCollector
{
public:
  std::list<can::Frame> frames;

  frameCollector() {}

  void frameCallback(const can::Frame & f)
  {
    frames.push_back(f);
  }
};

TEST(TopicToSocketCANTest, checkCorrectData)
{
  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  can::DummyInterfaceSharedPtr driver_ = std::make_shared<can::DummyInterface>(true);

  // start the to topic bridge.
  socketcan_bridge::TopicToSocketCAN to_socketcan_bridge(&nh, &nh_param, driver_);
  to_socketcan_bridge.setup();

  // init the driver to test stateListener (not checked automatically).
  driver_->init("string_not_used", true);

  // register for messages on received_messages.
  ros::Publisher publisher_ = nh.advertise<can_msgs::msg::Frame>("sent_messages", 10);

  // create a frame collector.
  frameCollector frame_collector_;

  //  driver->createMsgListener(&frameCallback);
  can::FrameListenerConstSharedPtr frame_listener_ = driver_->createMsgListener(
    std::bind(&frameCollector::frameCallback, &frame_collector_, std::placeholders::_1));

  // create a message
  can_msgs::msg::Frame msg;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.is_error = false;
  msg.id = 0x1337;
  msg.dlc = 8;

  for (uint8_t i = 0; i < msg.dlc; i++) {
    msg.data[i] = i;
  }

  msg.header.frame_id = "0";  // "0" for no frame.
  msg.header.stamp = ros::Time::now();

  // send the can_frame::Frame message to the sent_messages topic.
  publisher_.publish(msg);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  can_msgs::msg::Frame received;
  can::Frame f = frame_collector_.frames.back();
  socketcan_bridge::convertSocketCANToMessage(f, received);

  EXPECT_EQ(received.id, msg.id);
  EXPECT_EQ(received.dlc, msg.dlc);
  EXPECT_EQ(received.is_extended, msg.is_extended);
  EXPECT_EQ(received.is_rtr, msg.is_rtr);
  EXPECT_EQ(received.is_error, msg.is_error);
  EXPECT_EQ(received.data, msg.data);
}

TEST(TopicToSocketCANTest, checkInvalidFrameHandling)
{
  // - tries to send a non-extended frame with an id larger than 11 bits.
  //   that should not be sent.
  // - verifies that sending one larger than 11 bits actually works.
  // - tries sending a message with a dlc > 8 bytes, this should not be sent.
  //   sending with 8 bytes is verified by the checkCorrectData testcase.

  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  can::DummyInterfaceSharedPtr driver_ = std::make_shared<can::DummyInterface>(true);

  // start the to topic bridge.
  socketcan_bridge::TopicToSocketCAN to_socketcan_bridge(&nh, &nh_param, driver_);
  to_socketcan_bridge.setup();

  // register for messages on received_messages.
  ros::Publisher publisher_ = nh.advertise<can_msgs::msg::Frame>("sent_messages", 10);

  // create a frame collector.
  frameCollector frame_collector_;

  //  add callback to the dummy interface.
  can::FrameListenerConstSharedPtr frame_listener_ = driver_->createMsgListener(
    std::bind(&frameCollector::frameCallback, &frame_collector_, std::placeholders::_1));

  // create a message
  can_msgs::msg::Frame msg;
  msg.is_extended = false;
  msg.id = (1 << 11) + 1;  // this is an illegal CAN packet... should not be sent.
  msg.header.frame_id = "0";  // "0" for no frame.
  msg.header.stamp = ros::Time::now();

  // send the can_frame::Frame message to the sent_messages topic.
  publisher_.publish(msg);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(frame_collector_.frames.size(), 0);

  msg.is_extended = true;
  msg.id = (1 << 11) + 1;  // now it should be alright.
  // send the can_frame::Frame message to the sent_messages topic.
  publisher_.publish(msg);
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(frame_collector_.frames.size(), 1);

  // wipe the frame queue.
  frame_collector_.frames.clear();

  // finally, check if frames with a dlc > 8 are discarded.
  msg.dlc = 10;
  publisher_.publish(msg);
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(frame_collector_.frames.size(), 0);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_to_topic");
  ros::NodeHandle nh;
  ros::WallDuration(1.0).sleep();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
