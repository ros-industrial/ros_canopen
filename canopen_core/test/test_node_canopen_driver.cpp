#include "canopen_core/node_interfaces/node_canopen_driver.hpp"
#include "canopen_core/driver_node.hpp"
#include "gtest/gtest.h"

TEST(NodeCanopenDriver, test_bad_sequence_configure)
{
  rclcpp::init(0, nullptr);
  auto node = new rclcpp::Node("Node");
  auto interface =
      new ros2_canopen::node_interfaces::NodeCanopenDriver<rclcpp::Node>(node);
  std::shared_ptr<lely::canopen::AsyncMaster> master;
  std::shared_ptr<lely::ev::Executor> executor;
  EXPECT_ANY_THROW(interface->configure());
  rclcpp::shutdown();
}

TEST(NodeCanopenDriver, test_good_sequence)
{
  rclcpp::init(0, nullptr);
  auto node = new rclcpp::Node("Node");
  auto interface =
      new ros2_canopen::node_interfaces::NodeCanopenDriver<rclcpp::Node>(node);
  std::shared_ptr<lely::canopen::AsyncMaster> master;
  std::shared_ptr<lely::ev::Executor> executor;
  interface->init();
  interface->configure();
  rclcpp::shutdown();
}

