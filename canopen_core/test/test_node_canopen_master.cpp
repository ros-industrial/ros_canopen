#include "canopen_core/node_interfaces/node_canopen_master.hpp"
#include "canopen_core/master_node.hpp"
#include "gtest/gtest.h"

TEST(NodeCanopenMaster, test_bad_sequence_configure)
{
  rclcpp::init(0, nullptr);
  auto node = new rclcpp::Node("Node");
  auto interface =
      new ros2_canopen::node_interfaces::NodeCanopenMaster<rclcpp::Node>(node);
  EXPECT_ANY_THROW(interface->configure());
  rclcpp::shutdown();
}

TEST(NodeCanopenMaster, test_good_sequence)
{
  rclcpp::init(0, nullptr);
  auto node = new rclcpp::Node("Node");
  auto interface =
      new ros2_canopen::node_interfaces::NodeCanopenMaster<rclcpp::Node>(node);
  rclcpp::shutdown();
}

TEST(NodeCanopenMaster, test_init)
{
  rclcpp::init(0, nullptr);
  
  auto node = std::make_shared<ros2_canopen::CanopenMaster>();
  auto node_interface = std::static_pointer_cast<ros2_canopen::CanopenMasterInterface>(node);
  EXPECT_THROW(node_interface->get_executor(), ros2_canopen::MasterException);
  rclcpp::shutdown();
}
