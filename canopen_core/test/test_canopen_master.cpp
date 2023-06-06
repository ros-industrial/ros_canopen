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

#include <chrono>
#include "canopen_core/master_node.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
using namespace std::chrono_literals;
using namespace ros2_canopen;
using namespace testing;

class MockNodeCanopenMaster : public ros2_canopen::node_interfaces::NodeCanopenMasterInterface
{
public:
  MOCK_METHOD(void, init, (), (override));
  MOCK_METHOD(void, configure, (), (override));
  MOCK_METHOD(void, activate, (), (override));
  MOCK_METHOD(void, deactivate, (), (override));
  MOCK_METHOD(void, cleanup, (), (override));
  MOCK_METHOD(void, shutdown, (), (override));
  MOCK_METHOD(std::shared_ptr<lely::canopen::AsyncMaster>, get_master, (), (override));
  MOCK_METHOD(std::shared_ptr<lely::ev::Executor>, get_executor, (), (override));
};

class MockCanopenMaster : public CanopenMaster
{
  friend class CanopenMasterTest;
  FRIEND_TEST(CanopenMasterTest, test_init);
};

class CanopenMasterTest : public testing::Test
{
public:
  std::shared_ptr<MockCanopenMaster> canopen_master;
  std::shared_ptr<MockNodeCanopenMaster> node_canopen_master;
  void SetUp() override
  {
    RCLCPP_INFO(rclcpp::get_logger("CanopenMasterTest"), "SetUp");
    rclcpp::init(0, nullptr);
    canopen_master = std::make_shared<MockCanopenMaster>();
    node_canopen_master = std::make_shared<MockNodeCanopenMaster>();
    canopen_master->node_canopen_master_ =
      std::static_pointer_cast<ros2_canopen::node_interfaces::NodeCanopenMasterInterface>(
        node_canopen_master);
  }

  void TearDown() override
  {
    RCLCPP_INFO(rclcpp::get_logger("CanopenMasterTest"), "TearDown");
    rclcpp::shutdown();
  }
};

TEST_F(CanopenMasterTest, test_init)
{
  EXPECT_CALL(*node_canopen_master, init()).Times(1);
  EXPECT_CALL(*node_canopen_master, configure()).Times(1);
  EXPECT_CALL(*node_canopen_master, activate()).Times(1);
  canopen_master->init();
}

TEST_F(CanopenMasterTest, test_get_node_base_interface)
{
  auto base_iface = canopen_master->get_node_base_interface();
  EXPECT_TRUE(base_iface);
}

TEST_F(CanopenMasterTest, test_shutdown)
{
  EXPECT_CALL(*node_canopen_master, shutdown());
  EXPECT_NO_THROW(canopen_master->shutdown());
}

TEST_F(CanopenMasterTest, test_is_lifecycle) { EXPECT_FALSE(canopen_master->is_lifecycle()); }

TEST_F(CanopenMasterTest, test_get_master)
{
  EXPECT_CALL(*node_canopen_master, get_master());
  EXPECT_NO_THROW(canopen_master->get_master());
}

TEST_F(CanopenMasterTest, test_get_executor)
{
  EXPECT_CALL(*node_canopen_master, get_executor());
  EXPECT_NO_THROW(canopen_master->get_executor());
}
