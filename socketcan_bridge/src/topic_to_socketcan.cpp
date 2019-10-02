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
#include <socketcan_interface/string.hpp>
#include <string>

namespace socketcan_bridge
{
  TopicToSocketCAN::TopicToSocketCAN(rclcpp::Node::SharedPtr node_ptr,
    can::DriverInterfaceSharedPtr driver) :
    node_ptr_(node_ptr),
    can_topic_(node_ptr_->create_subscription<can_msgs::msg::Frame>(
      "sent_messages", 10, std::bind(&TopicToSocketCAN::msgCallback, this, std::placeholders::_1))),
    driver_(driver)
    {}

  void TopicToSocketCAN::setup()
  {
    state_listener_ = driver_->createStateListener(
      std::bind(&TopicToSocketCAN::stateCallback, this, std::placeholders::_1));
  };

  void TopicToSocketCAN::msgCallback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    // ROS_DEBUG("Message came from sent_messages topic");

    // translate it to the socketcan frame type.

    can::Frame f;  // socketcan type

    // converts the can_msgs::msg::Frame (ROS msg) to can::Frame (socketcan.h)
    convertMessageToSocketCAN(msg, f);

    if (!f.isValid())  // check if the id and flags are appropriate.
    {
      // ROS_WARN("Refusing to send invalid frame: %s.", can::tostring(f, true).c_str());
      // can::tostring cannot be used for dlc > 8 frames. It causes an crash
      // due to usage of boost::array for the data array. The should always work.
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Invalid frame from topic: id: %#04x, length: %d, is_extended: %d",
        msg->id, msg->dlc, msg->is_extended);
      return;
    }

    bool res = driver_->send(f);
    if (!res)
    {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Failed to send message: %s.",
        can::tostring(f, true).c_str());
    }
  }

  void TopicToSocketCAN::stateCallback(const can::State & s)
  {
    std::string err;
    driver_->translateError(s.internal_error, err);
    if (!s.internal_error)
    {
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "State: %s, asio: %s",
        err.c_str(),
        s.error_code.message().c_str());
    }
    else
    {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Error: %s, asio: %s",
        err.c_str(),
        s.error_code.message().c_str());
    }
  }
};  // namespace socketcan_bridge
