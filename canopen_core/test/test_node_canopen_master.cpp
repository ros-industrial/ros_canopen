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

#include "canopen_core/master_node.hpp"
#include "canopen_core/node_interfaces/node_canopen_master.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using namespace ros2_canopen;
using namespace ros2_canopen::node_interfaces;
using namespace testing;
template <class NODETYPE>
class MockNodeCanopenMaster : public NodeCanopenMaster<NODETYPE>
{
public:
  FRIEND_TEST(NodeCanopenMasterTest, test_construct);
  FRIEND_TEST(NodeCanopenMasterTest, test_init);
  FRIEND_TEST(NodeCanopenMasterTest, test_init_fail_configured);
  FRIEND_TEST(NodeCanopenMasterTest, test_init_fail_activated);
  FRIEND_TEST(NodeCanopenMasterTest, test_set_master);
  FRIEND_TEST(NodeCanopenMasterTest, test_configure);
  FRIEND_TEST(NodeCanopenMasterTest, test_configure_fail_not_initialsed);
  FRIEND_TEST(NodeCanopenMasterTest, test_configure_fail_configured);
  FRIEND_TEST(NodeCanopenMasterTest, test_configure_fail_activated);
  FRIEND_TEST(NodeCanopenMasterTest, test_activate);
  FRIEND_TEST(NodeCanopenMasterTest, test_activate_failures);
  FRIEND_TEST(NodeCanopenMasterTest, test_deactivate);
  FRIEND_TEST(NodeCanopenMasterTest, test_deactivate_failures);
  FRIEND_TEST(NodeCanopenMasterTest, test_cleanup);
  FRIEND_TEST(NodeCanopenMasterTest, test_cleanup_failures);
  FRIEND_TEST(NodeCanopenMasterTest, test_shutdown);
  FRIEND_TEST(NodeCanopenMasterTest, test_get_master);
  FRIEND_TEST(NodeCanopenMasterTest, test_get_executor);

  FRIEND_TEST(NodeCanopenLMasterTest, test_construct);
  FRIEND_TEST(NodeCanopenLMasterTest, test_init);
  FRIEND_TEST(NodeCanopenLMasterTest, test_init_fail_configured);
  FRIEND_TEST(NodeCanopenLMasterTest, test_init_fail_activated);
  FRIEND_TEST(NodeCanopenLMasterTest, test_set_master);
  FRIEND_TEST(NodeCanopenLMasterTest, test_configure);
  FRIEND_TEST(NodeCanopenLMasterTest, test_configure_fail_not_initialsed);
  FRIEND_TEST(NodeCanopenLMasterTest, test_configure_fail_configured);
  FRIEND_TEST(NodeCanopenLMasterTest, test_configure_fail_activated);
  FRIEND_TEST(NodeCanopenLMasterTest, test_activate);
  FRIEND_TEST(NodeCanopenLMasterTest, test_activate_failures);
  FRIEND_TEST(NodeCanopenLMasterTest, test_deactivate);
  FRIEND_TEST(NodeCanopenLMasterTest, test_deactivate_failures);
  FRIEND_TEST(NodeCanopenLMasterTest, test_cleanup);
  FRIEND_TEST(NodeCanopenLMasterTest, test_cleanup_failures);
  FRIEND_TEST(NodeCanopenLMasterTest, test_shutdown);
  FRIEND_TEST(NodeCanopenLMasterTest, test_get_master);
  FRIEND_TEST(NodeCanopenLMasterTest, test_get_executor);

  MockNodeCanopenMaster(NODETYPE * node) : NodeCanopenMaster<NODETYPE>(node) {}

  void DelegateToBase()
  {
    // ON_CALL(*this, load_component)
    //     .WillByDefault(
    //         [this](
    //             std::string & package_name, std::string & driver_name, uint16_t node_id,
    //             std::string & node_name, std::vector<rclcpp::Parameter> & params) -> bool
    //         { return DeviceContainer::load_component(package_name, driver_name, node_id,
    //         node_name, params); });
  }
};

class NodeCanopenMasterTest : public testing::Test
{
public:
  std::shared_ptr<MockNodeCanopenMaster<rclcpp::Node>> node_canopen_master;
  rclcpp::Node::SharedPtr node;
  void SetUp() override
  {
    RCLCPP_INFO(rclcpp::get_logger("CanopenMasterTest"), "SetUp");
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("driver");
    node_canopen_master = std::make_shared<MockNodeCanopenMaster<rclcpp::Node>>(node.get());
    node_canopen_master->DelegateToBase();
  }

  void TearDown() override
  {
    RCLCPP_INFO(rclcpp::get_logger("CanopenMasterTest"), "TearDown");
    rclcpp::shutdown();
  }
};

TEST_F(NodeCanopenMasterTest, test_construct) { EXPECT_EQ(node.get(), node_canopen_master->node_); }

TEST_F(NodeCanopenMasterTest, test_init)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  node_canopen_master->init();
  EXPECT_TRUE(node_canopen_master->initialised_.load());
  EXPECT_TRUE(node->has_parameter("container_name"));
  EXPECT_TRUE(node->has_parameter("node_id"));
  EXPECT_TRUE(node->has_parameter("non_transmit_timeout"));
  EXPECT_TRUE(node->has_parameter("config"));
}

TEST_F(NodeCanopenMasterTest, test_init_fail_configured)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->init(), MasterException);
  EXPECT_FALSE(node_canopen_master->initialised_.load());
  EXPECT_FALSE(node->has_parameter("container_name"));
  EXPECT_FALSE(node->has_parameter("node_id"));
  EXPECT_FALSE(node->has_parameter("non_transmit_timeout"));
  EXPECT_FALSE(node->has_parameter("config"));
}

TEST_F(NodeCanopenMasterTest, test_init_fail_activated)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->init(), MasterException);
  EXPECT_FALSE(node_canopen_master->initialised_.load());
  EXPECT_FALSE(node->has_parameter("container_name"));
  EXPECT_FALSE(node->has_parameter("node_id"));
  EXPECT_FALSE(node->has_parameter("non_transmit_timeout"));
  EXPECT_FALSE(node->has_parameter("config"));
}

TEST_F(NodeCanopenMasterTest, test_configure)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  node_canopen_master->init();
  node->set_parameter(rclcpp::Parameter(
    "config", "node_id: 1\ndriver: \"ros2_canopen::CanopenMaster\"\npackage:\"canopen_core\"\n"));
  node_canopen_master->configure();
  EXPECT_TRUE(node_canopen_master->configured_.load());
}

TEST_F(NodeCanopenMasterTest, test_configure_fail_not_initialsed)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->configure(), MasterException);
  EXPECT_FALSE(node_canopen_master->configured_.load());
}

TEST_F(NodeCanopenMasterTest, test_configure_fail_configured)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  node_canopen_master->init();
  node_canopen_master->configured_.store(true);
  node->set_parameter(rclcpp::Parameter(
    "config", "node_id: 1\ndriver: \"ros2_canopen::CanopenDriver\"\npackage:\"canopen_core\"\n"));
  EXPECT_THROW(node_canopen_master->configure(), MasterException);
}

TEST_F(NodeCanopenMasterTest, test_configure_fail_activated)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  node_canopen_master->init();
  node_canopen_master->activated_.store(true);
  node->set_parameter(rclcpp::Parameter(
    "config", "node_id: 1\ndriver: \"ros2_canopen::CanopenDriver\"\npackage:\"canopen_core\"\n"));
  EXPECT_THROW(node_canopen_master->configure(), MasterException);
  EXPECT_FALSE(node_canopen_master->configured_.load());
}

TEST_F(NodeCanopenMasterTest, test_activate_failures)
{
  node_canopen_master->initialised_.store(false);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->activate(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->activate(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->activate(), MasterException);
}

TEST_F(NodeCanopenMasterTest, test_deactivate_failures)
{
  node_canopen_master->initialised_.store(false);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->deactivate(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->deactivate(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->deactivate(), MasterException);
}

TEST_F(NodeCanopenMasterTest, test_cleanup)
{
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(false);
  node_canopen_master->cleanup();
  EXPECT_FALSE(node_canopen_master->configured_.load());
}

TEST_F(NodeCanopenMasterTest, test_cleanup_failures)
{
  node_canopen_master->initialised_.store(false);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->cleanup(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->cleanup(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->cleanup(), MasterException);
}

TEST_F(NodeCanopenMasterTest, test_shutdown)
{
  node_canopen_master->master_set_.store(true);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  EXPECT_NO_THROW(node_canopen_master->shutdown());
  EXPECT_FALSE(node_canopen_master->master_set_.load());
  EXPECT_FALSE(node_canopen_master->initialised_.load());
  EXPECT_FALSE(node_canopen_master->configured_.load());
  EXPECT_FALSE(node_canopen_master->activated_.load());
}

TEST_F(NodeCanopenMasterTest, test_get_master)
{
  node_canopen_master->master_set_.store(true);
  EXPECT_NO_THROW(node_canopen_master->get_master());

  node_canopen_master->master_set_.store(false);
  EXPECT_ANY_THROW(node_canopen_master->get_master());
}

TEST_F(NodeCanopenMasterTest, test_get_executor)
{
  node_canopen_master->master_set_.store(true);
  EXPECT_NO_THROW(node_canopen_master->get_executor());

  node_canopen_master->master_set_.store(false);
  EXPECT_ANY_THROW(node_canopen_master->get_executor());
}

class NodeCanopenLMasterTest : public testing::Test
{
public:
  std::shared_ptr<MockNodeCanopenMaster<rclcpp_lifecycle::LifecycleNode>> node_canopen_master;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  void SetUp() override
  {
    RCLCPP_INFO(rclcpp::get_logger("CanopenMasterTest"), "SetUp");
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("driver");
    node_canopen_master =
      std::make_shared<MockNodeCanopenMaster<rclcpp_lifecycle::LifecycleNode>>(node.get());
    node_canopen_master->DelegateToBase();
  }

  void TearDown() override
  {
    RCLCPP_INFO(rclcpp::get_logger("CanopenMasterTest"), "TearDown");
    rclcpp::shutdown();
  }
};

TEST_F(NodeCanopenLMasterTest, test_construct)
{
  EXPECT_EQ(node.get(), node_canopen_master->node_);
}

TEST_F(NodeCanopenLMasterTest, test_init)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  node_canopen_master->init();
  EXPECT_TRUE(node_canopen_master->initialised_.load());
  EXPECT_TRUE(node->has_parameter("container_name"));
  EXPECT_TRUE(node->has_parameter("node_id"));
  EXPECT_TRUE(node->has_parameter("non_transmit_timeout"));
  EXPECT_TRUE(node->has_parameter("config"));
}

TEST_F(NodeCanopenLMasterTest, test_init_fail_configured)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->init(), MasterException);
  EXPECT_FALSE(node_canopen_master->initialised_.load());
  EXPECT_FALSE(node->has_parameter("container_name"));
  EXPECT_FALSE(node->has_parameter("node_id"));
  EXPECT_FALSE(node->has_parameter("non_transmit_timeout"));
  EXPECT_FALSE(node->has_parameter("config"));
}

TEST_F(NodeCanopenLMasterTest, test_init_fail_activated)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->init(), MasterException);
  EXPECT_FALSE(node_canopen_master->initialised_.load());
  EXPECT_FALSE(node->has_parameter("container_name"));
  EXPECT_FALSE(node->has_parameter("node_id"));
  EXPECT_FALSE(node->has_parameter("non_transmit_timeout"));
  EXPECT_FALSE(node->has_parameter("config"));
}

TEST_F(NodeCanopenLMasterTest, test_configure)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  node_canopen_master->init();
  node->set_parameter(rclcpp::Parameter(
    "config", "node_id: 1\ndriver: \"ros2_canopen::CanopenMaster\"\npackage:\"canopen_core\"\n"));
  node_canopen_master->configure();
  EXPECT_TRUE(node_canopen_master->configured_.load());
}

TEST_F(NodeCanopenLMasterTest, test_configure_fail_not_initialsed)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->configure(), MasterException);
  EXPECT_FALSE(node_canopen_master->configured_.load());
}

TEST_F(NodeCanopenLMasterTest, test_configure_fail_configured)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  node_canopen_master->init();
  node_canopen_master->configured_.store(true);
  node->set_parameter(rclcpp::Parameter(
    "config", "node_id: 1\ndriver: \"ros2_canopen::CanopenDriver\"\npackage:\"canopen_core\"\n"));
  EXPECT_THROW(node_canopen_master->configure(), MasterException);
}

TEST_F(NodeCanopenLMasterTest, test_configure_fail_activated)
{
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  node_canopen_master->init();
  node_canopen_master->activated_.store(true);
  node->set_parameter(rclcpp::Parameter(
    "config", "node_id: 1\ndriver: \"ros2_canopen::CanopenDriver\"\npackage:\"canopen_core\"\n"));
  EXPECT_THROW(node_canopen_master->configure(), MasterException);
  EXPECT_FALSE(node_canopen_master->configured_.load());
}

TEST_F(NodeCanopenLMasterTest, test_activate_failures)
{
  node_canopen_master->initialised_.store(false);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->activate(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->activate(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->activate(), MasterException);
}

TEST_F(NodeCanopenLMasterTest, test_deactivate_failures)
{
  node_canopen_master->initialised_.store(false);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->deactivate(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->deactivate(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->deactivate(), MasterException);
}

TEST_F(NodeCanopenLMasterTest, test_cleanup)
{
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(false);
  node_canopen_master->cleanup();
  EXPECT_FALSE(node_canopen_master->configured_.load());
}

TEST_F(NodeCanopenLMasterTest, test_cleanup_failures)
{
  node_canopen_master->initialised_.store(false);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->cleanup(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  EXPECT_THROW(node_canopen_master->cleanup(), MasterException);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(true);
  node_canopen_master->activated_.store(true);
  EXPECT_THROW(node_canopen_master->cleanup(), MasterException);
}

TEST_F(NodeCanopenLMasterTest, test_shutdown)
{
  node_canopen_master->master_set_.store(true);
  node_canopen_master->initialised_.store(true);
  node_canopen_master->configured_.store(false);
  node_canopen_master->activated_.store(false);
  EXPECT_NO_THROW(node_canopen_master->shutdown());
  EXPECT_FALSE(node_canopen_master->master_set_.load());
  EXPECT_FALSE(node_canopen_master->initialised_.load());
  EXPECT_FALSE(node_canopen_master->configured_.load());
  EXPECT_FALSE(node_canopen_master->activated_.load());
}

TEST_F(NodeCanopenLMasterTest, test_get_master)
{
  node_canopen_master->master_set_.store(true);
  EXPECT_NO_THROW(node_canopen_master->get_master());

  node_canopen_master->master_set_.store(false);
  EXPECT_ANY_THROW(node_canopen_master->get_master());
}

TEST_F(NodeCanopenLMasterTest, test_get_executor)
{
  node_canopen_master->master_set_.store(true);
  EXPECT_NO_THROW(node_canopen_master->get_executor());

  node_canopen_master->master_set_.store(false);
  EXPECT_ANY_THROW(node_canopen_master->get_executor());
}
