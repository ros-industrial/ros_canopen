#include "canopen_base_driver/node_interfaces/node_canopen_base_driver.hpp"
#include "gtest/gtest.h"
#include <thread>
#include <rclcpp/executors.hpp>

TEST(NodeCanopenBaseDriver, test_good_sequence_advanced)
{
  rclcpp::init(0, nullptr);
  rclcpp::Node *node = new rclcpp::Node("Node");
  auto interface =
      new ros2_canopen::node_interfaces::NodeCanopenBaseDriver(node);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node->get_node_base_interface());
  std::thread spinner = std::thread([exec]
                        { 
                          exec->spin();
                        });

  auto iface = static_cast<ros2_canopen::node_interfaces::NodeCanopenDriverInterface*>(interface);

  EXPECT_NO_THROW(iface->init());

  rclcpp::Parameter container_name("container_name", "none");
  rclcpp::Parameter node_id("node_id", 1);
  rclcpp::Parameter timeout("non_transmit_timeout", 100);
  rclcpp::Parameter config("config", "");
  node->set_parameter(container_name);
  node->set_parameter(node_id);
  node->set_parameter(timeout);
  node->set_parameter(config);
  EXPECT_NO_THROW(iface->configure());
  // Can't activate as master cannot be set.
  EXPECT_ANY_THROW(iface->activate());
  iface->shutdown();
  rclcpp::shutdown();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if(spinner.joinable())
  {
    spinner.join();
  }
}


TEST(NodeCanopenBasicLifecycleMaster, test_good_sequence_advanced)
{
  rclcpp::init(0, nullptr);
  rclcpp_lifecycle::LifecycleNode *node = new rclcpp_lifecycle::LifecycleNode("Node");
  auto interface =
      new ros2_canopen::node_interfaces::NodeCanopenBaseDriver(node);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node->get_node_base_interface());
  std::thread spinner = std::thread([exec]
                        { 
                          exec->spin();
                        });

  auto iface = static_cast<ros2_canopen::node_interfaces::NodeCanopenDriverInterface *>(interface);

  EXPECT_NO_THROW(iface->init());

  rclcpp::Parameter container_name("container_name", "none");
  rclcpp::Parameter node_id("node_id", 1);
  rclcpp::Parameter timeout("non_transmit_timeout", 100);
  rclcpp::Parameter config("config", "");
  node->set_parameter(container_name);
  node->set_parameter(node_id);
  node->set_parameter(timeout);
  node->set_parameter(config);
  EXPECT_NO_THROW(iface->configure());
  // Can't activate as master cannot be set.
  EXPECT_ANY_THROW(iface->activate());
  rclcpp::shutdown();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if(spinner.joinable())
  {
    spinner.join();
  }
}
