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
#include <socketcan_interface/string.hpp>
#include <can_msgs/msg/frame.hpp>

#include <memory>
#include <string>

/*
 * TODO: Find replacement for XmlRpc stuff
namespace can {
template<> can::FrameFilterSharedPtr tofilter(const XmlRpc::XmlRpcValue  &ct) {
  XmlRpc::XmlRpcValue t(ct);
  try{ // to read as integer
      uint32_t id = static_cast<int>(t);
      return tofilter(id);
  }
  catch(...){ // read as string
      return  tofilter(static_cast<std::string>(t));
  }
}
}
*/

namespace socketcan_bridge
{
SocketCANToTopic::SocketCANToTopic(
  rclcpp::Node::SharedPtr node_ptr,
  can::DriverInterfaceSharedPtr driver)
: node_ptr_(node_ptr),
  can_topic_(node_ptr_->create_publisher<can_msgs::msg::Frame>("received_messages", 10)),
  driver_(driver)
{}

void SocketCANToTopic::setup()
{
  // register handler for frames and state changes.
  frame_listener_ = driver_->createMsgListenerM(this, &SocketCANToTopic::frameCallback);
  state_listener_ = driver_->createStateListenerM(this, &SocketCANToTopic::stateCallback);
}

void SocketCANToTopic::setup(const can::FilteredFrameListener::FilterVector & filters)
{
  frame_listener_.reset(
    new can::FilteredFrameListener(
      driver_, std::bind(
        &SocketCANToTopic::frameCallback, this, std::placeholders::_1),
      filters));

  state_listener_ = driver_->createStateListenerM(this, &SocketCANToTopic::stateCallback);
}

/*
void SocketCANToTopic::setup(XmlRpc::XmlRpcValue filters) {
    setup(can::tofilters(filters));
}
*/
void SocketCANToTopic::setup(rclcpp::Node::SharedPtr node_ptr)
{
  // XmlRpc::XmlRpcValue filters;
  // if(node_ptr->get_parameter("can_ids", filters)) return setup(filters);
  setup();
}

void SocketCANToTopic::frameCallback(const can::Frame & f)
{
  // ROS_DEBUG("Message came in: %s", can::tostring(f, true).c_str());
  if (!f.isValid()) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "Invalid frame from SocketCAN: id: %#04x, length: %d, is_extended: %d,"
      "is_error: %d, is_rtr: %d",
      f.id, f.dlc, f.is_extended, f.is_error, f.is_rtr);
    return;
  } else {
    if (f.is_error) {
      // can::tostring cannot be used for dlc > 8 frames. It causes an crash
      // due to usage of boost::array for the data array. The should always work.
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Received frame is error: %s",
        can::tostring(f, true).c_str());
    }
  }

  auto msg = std::make_shared<can_msgs::msg::Frame>();
  // converts the can::Frame (socketcan.h) to can_msgs::msg::Frame (ROS msg)
  convertSocketCANToMessage(f, *msg);

  msg->header.frame_id = "";  // empty frame is the de-facto standard for no frame.
  msg->header.stamp = node_ptr_->now();

  can_topic_->publish(*msg);
}

void SocketCANToTopic::stateCallback(const can::State & s)
{
  std::string err;
  driver_->translateError(s.internal_error, err);
  if (!s.internal_error) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "State: %s, asio: %s",
      err.c_str(),
      s.error_code.message().c_str());
  } else {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "Error: %s, asio: %s",
      err.c_str(),
      s.error_code.message().c_str());
  }
}
}  // namespace socketcan_bridge
