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

#ifndef SOCKETCAN_BRIDGE__SOCKETCAN_TO_TOPIC_HPP_
#define SOCKETCAN_BRIDGE__SOCKETCAN_TO_TOPIC_HPP_

#include <socketcan_interface/socketcan.hpp>
#include <socketcan_interface/filter.hpp>
#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace socketcan_bridge
{
class SocketCANToTopic
{
public:
  SocketCANToTopic(rclcpp::Node::SharedPtr node_ptr, can::DriverInterfaceSharedPtr driver);
  void setup();
  void setup(const can::FilteredFrameListener::FilterVector & filters);
  // void setup(XmlRpc::XmlRpcValue filters);  // TODO: Find replacement for XmlRpc filters.
  void setup(rclcpp::Node::SharedPtr node_ptr);

private:
  rclcpp::Node::SharedPtr node_ptr_;
  std::shared_ptr<rclcpp::Publisher<can_msgs::msg::Frame>> can_topic_;
  can::DriverInterfaceSharedPtr driver_;

  can::FrameListenerConstSharedPtr frame_listener_;
  can::StateListenerConstSharedPtr state_listener_;

  void frameCallback(const can::Frame & f);
  void stateCallback(const can::State & s);
};

void convertSocketCANToMessage(const can::Frame & f, can_msgs::msg::Frame & m)
{
  m.id = f.id;
  m.dlc = f.dlc;
  m.is_error = f.is_error;
  m.is_rtr = f.is_rtr;
  m.is_extended = f.is_extended;

  for (int i = 0; i < 8; i++) {  // always copy all data, regardless of dlc.
    m.data[i] = f.data[i];
  }
}
}  // namespace socketcan_bridge

#endif  // SOCKETCAN_BRIDGE__SOCKETCAN_TO_TOPIC_HPP_
