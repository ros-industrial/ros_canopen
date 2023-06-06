//    Copyright 2022 Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#include "canopen_master_driver/node_interfaces/node_canopen_basic_master.hpp"
#include "gtest/gtest.h"

class RclCppFixture
{
public:
  RclCppFixture()
  {
    rclcpp::init(0, nullptr);
    node = new rclcpp::Node("Node");
    interface = new ros2_canopen::node_interfaces::NodeCanopenBasicMaster<rclcpp::Node>(node);
  }
  ~RclCppFixture()
  {
    rclcpp::shutdown();
    delete (interface);
    delete (node);
  }

  rclcpp::Node * node;
  ros2_canopen::node_interfaces::NodeCanopenBasicMaster<rclcpp::Node> * interface;
};
RclCppFixture g_rclcppfixture;

TEST(NodeCanopenBasicMaster, test_bad_sequence_configure)
{
  auto iface = static_cast<ros2_canopen::node_interfaces::NodeCanopenMasterInterface *>(
    g_rclcppfixture.interface);
  EXPECT_ANY_THROW(iface->configure());
}

TEST(NodeCanopenBasicMaster, test_bad_sequence_activate)
{
  auto iface = static_cast<ros2_canopen::node_interfaces::NodeCanopenMasterInterface *>(
    g_rclcppfixture.interface);
  EXPECT_ANY_THROW(iface->activate());
}

TEST(NodeCanopenBasicMaster, test_good_sequence)
{
  auto iface = static_cast<ros2_canopen::node_interfaces::NodeCanopenMasterInterface *>(
    g_rclcppfixture.interface);
  try
  {
    iface->init();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("test"), e.what());
  }
  EXPECT_NO_THROW(iface->configure());
}
