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

#ifndef SOCKETCAN_BRIDGE__TOPIC_TO_SOCKETCAN_HPP_
#define SOCKETCAN_BRIDGE__TOPIC_TO_SOCKETCAN_HPP_

#include <socketcan_interface/socketcan.hpp>
#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>

namespace socketcan_bridge
{
class TopicToSocketCAN
{
public:
  TopicToSocketCAN(rclcpp::Node::SharedPtr node_ptr, can::DriverInterfaceSharedPtr driver);
  void setup();

private:
  rclcpp::Node::SharedPtr node_ptr_;
  can::DriverInterfaceSharedPtr driver_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_topic_;

  can::StateListenerConstSharedPtr state_listener_;

  void msgCallback(const can_msgs::msg::Frame::SharedPtr msg);
  void stateCallback(const can::State & s);
};

void convertMessageToSocketCAN(
  const can_msgs::msg::Frame::SharedPtr m, can::Frame & f)
{
  f.id = m->id;
  f.dlc = m->dlc;
  f.is_error = m->is_error;
  f.is_rtr = m->is_rtr;
  f.is_extended = m->is_extended;

  for (int i = 0; i < 8; i++) {  // always copy all data, regardless of dlc.
    f.data[i] = m->data[i];
  }
}
};  // namespace socketcan_bridge

#endif  // SOCKETCAN_BRIDGE__TOPIC_TO_SOCKETCAN_HPP_
