#include "canopen_core/node_interfaces/node_canopen_driver.hpp"
#include "gtest/gtest.h"

class RclCppFixture
{
public:
  RclCppFixture()
  {
    rclcpp::init(0, nullptr);
    node = new rclcpp::Node("Node");
    interface =
        new ros2_canopen::node_interfaces::NodeCanopenDriver<rclcpp::Node>(node);
  }
  ~RclCppFixture()
  {
    rclcpp::shutdown();
    delete(interface);
    delete(node);
    
  }

  rclcpp::Node *node;
  ros2_canopen::node_interfaces::NodeCanopenDriver<rclcpp::Node> *interface;
};
RclCppFixture g_rclcppfixture;

TEST(NodeCanopenDriver, test_bad_sequence_configure)
{
  std::shared_ptr<lely::canopen::AsyncMaster> master;
  std::shared_ptr<lely::ev::Executor> executor;
  EXPECT_ANY_THROW(g_rclcppfixture.interface->configure());
}

TEST(NodeCanopenDriver, test_bad_sequence_activate)
{
  std::shared_ptr<lely::canopen::AsyncMaster> master;
  std::shared_ptr<lely::ev::Executor> executor;
  EXPECT_ANY_THROW(g_rclcppfixture.interface->activate());
}

TEST(NodeCanopenDriver, test_good_sequence)
{
  std::shared_ptr<lely::canopen::AsyncMaster> master;
  std::shared_ptr<lely::ev::Executor> executor;
  g_rclcppfixture.interface->init();
  g_rclcppfixture.interface->configure();
  g_rclcppfixture.interface->set_master(executor, master);
  g_rclcppfixture.interface->activate();
  g_rclcppfixture.interface->deactivate();
  g_rclcppfixture.interface->cleanup();
}


