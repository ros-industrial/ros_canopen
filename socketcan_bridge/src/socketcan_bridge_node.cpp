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

#include <rclcpp/rclcpp.hpp>
#include <socketcan_bridge/topic_to_socketcan.hpp>
#include <socketcan_bridge/socketcan_to_topic.hpp>
#include <socketcan_interface/threading.hpp>
#include <string>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("socketcan_bridge_node");

  std::string can_device;
  node->get_parameter_or<std::string>("can_device", can_device, "can0");

  can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface> ();

  if (!driver->init(can_device, 0))  // initialize device at can_device, 0 for no loopback.
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize can_device at %s", can_device.c_str());
    return 1;
  }
    else
  {
    RCLCPP_INFO(node->get_logger(), "Successfully connected to %s.", can_device.c_str());
  }

  // initialize the bridge both ways.
  socketcan_bridge::TopicToSocketCAN to_socketcan_bridge(node, driver);
  to_socketcan_bridge.setup();

  socketcan_bridge::SocketCANToTopic to_topic_bridge(node, driver);
  to_topic_bridge.setup(node);

  rclcpp::spin(node);

  driver->shutdown();
  driver.reset();
}
