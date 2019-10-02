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

#include <socketcan_bridge/topic_to_socketcan.hpp>
#include <socketcan_bridge/socketcan_to_topic.hpp>

#include <can_msgs/msg/frame.hpp>
#include <socketcan_interface/socketcan.hpp>

#include <gtest/gtest.h>

#include <memory>

// test whether the content of a conversion from a SocketCAN frame
// to a ROS message correctly maintains the data.
TEST(ConversionTest, socketCANToTopicStandard)
{
  can::Frame f;
  auto m = can_msgs::msg::Frame();
  f.id = 127;
  f.dlc = 8;
  f.is_error = false;
  f.is_rtr = false;
  f.is_extended = false;
  for (uint8_t i = 0; i < f.dlc; ++i)
  {
    f.data[i] = i;
  }
  socketcan_bridge::convertSocketCANToMessage(f, m);
  EXPECT_EQ(127, m.id);
  EXPECT_EQ(8, m.dlc);
  EXPECT_EQ(false, m.is_error);
  EXPECT_EQ(false, m.is_rtr);
  EXPECT_EQ(false, m.is_extended);

  for (uint8_t i=0; i < 8; i++)
  {
    EXPECT_EQ(i, m.data[i]);
  }
}

// test all three flags seperately.
TEST(ConversionTest, socketCANToTopicFlags)
{
  can::Frame f;
  auto m = can_msgs::msg::Frame();

  f.is_error = true;
  socketcan_bridge::convertSocketCANToMessage(f, m);
  EXPECT_EQ(true, m.is_error);
  f.is_error = false;

  f.is_rtr = true;
  socketcan_bridge::convertSocketCANToMessage(f, m);
  EXPECT_EQ(true, m.is_rtr);
  f.is_rtr = false;

  f.is_extended = true;
  socketcan_bridge::convertSocketCANToMessage(f, m);
  EXPECT_EQ(true, m.is_extended);
  f.is_extended = false;
}

// idem, but the other way around.
TEST(ConversionTest, topicToSocketCANStandard)
{
  can::Frame f;
  auto m = can_msgs::msg::Frame();
  m.id = 127;
  m.dlc = 8;
  m.is_error = false;
  m.is_rtr = false;
  m.is_extended = false;
  for (uint8_t i = 0; i < m.dlc; ++i)
  {
    m.data[i] = i;
  }
  socketcan_bridge::convertMessageToSocketCAN(std::make_shared<can_msgs::msg::Frame>(m), f);
  EXPECT_EQ(127, f.id);
  EXPECT_EQ(8, f.dlc);
  EXPECT_EQ(false, f.is_error);
  EXPECT_EQ(false, f.is_rtr);
  EXPECT_EQ(false, f.is_extended);


  for (uint8_t i=0; i < 8; i++)
  {
    EXPECT_EQ(i, f.data[i]);
  }
}

TEST(ConversionTest, topicToSocketCANFlags)
{
  can::Frame f;
  auto m = can_msgs::msg::Frame();

  m.is_error = true;
  socketcan_bridge::convertMessageToSocketCAN(
      std::make_shared<can_msgs::msg::Frame>(m), f);
  EXPECT_EQ(true, f.is_error);
  m.is_error = false;

  m.is_rtr = true;
  socketcan_bridge::convertMessageToSocketCAN(
      std::make_shared<can_msgs::msg::Frame>(m), f);
  EXPECT_EQ(true, f.is_rtr);
  m.is_rtr = false;

  m.is_extended = true;
  socketcan_bridge::convertMessageToSocketCAN(
      std::make_shared<can_msgs::msg::Frame>(m), f);
  EXPECT_EQ(true, f.is_extended);
  m.is_extended = false;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
