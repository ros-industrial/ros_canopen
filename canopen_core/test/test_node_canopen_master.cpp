#include "canopen_core/node_interfaces/node_canopen_master.hpp"
#include "gtest/gtest.h"

class RclCppFixture
{
public:
  RclCppFixture()
  {
    rclcpp::init(0, nullptr);
    node = new rclcpp::Node("Node");
    interface =
        new ros2_canopen::node_interfaces::NodeCanopenMaster<rclcpp::Node>(node);
  }
  ~RclCppFixture()
  {
    rclcpp::shutdown();
    delete(interface);
    delete(node);
    
  }

  rclcpp::Node *node;
  ros2_canopen::node_interfaces::NodeCanopenMaster<rclcpp::Node> *interface;
};
RclCppFixture g_rclcppfixture;

TEST(NodeCanopenMaster, test_bad_sequence_configure)
{
  EXPECT_ANY_THROW(g_rclcppfixture.interface->configure());
}

TEST(NodeCanopenMaster, test_bad_sequence_activate)
{
  EXPECT_ANY_THROW(g_rclcppfixture.interface->activate());
}

TEST(NodeCanopenMaster, test_good_sequence)
{
  EXPECT_NO_THROW(g_rclcppfixture.interface->init());
  EXPECT_NO_THROW(g_rclcppfixture.interface->configure());
}


