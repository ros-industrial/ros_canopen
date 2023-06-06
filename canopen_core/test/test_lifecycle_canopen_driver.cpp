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
#include "canopen_core/driver_node.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
using namespace std::chrono_literals;
using namespace ros2_canopen;
using namespace testing;

class MockNodeCanopenMaster : public ros2_canopen::node_interfaces::NodeCanopenDriverInterface
{
public:
  MOCK_METHOD(
    void, set_master,
    (std::shared_ptr<lely::ev::Executor> exec, std::shared_ptr<lely::canopen::AsyncMaster> master),
    (override));
  MOCK_METHOD(void, demand_set_master, (), (override));
  MOCK_METHOD(void, init, (), (override));
  MOCK_METHOD(void, configure, (), (override));
  MOCK_METHOD(void, activate, (), (override));
  MOCK_METHOD(void, deactivate, (), (override));
  MOCK_METHOD(void, cleanup, (), (override));
  MOCK_METHOD(void, shutdown, (), (override));
  MOCK_METHOD(void, add_to_master, (), (override));
  MOCK_METHOD(void, remove_from_master, (), (override));
};

class MockCanopenMaster : public LifecycleCanopenDriver
{
  friend class CanopenDriverTest;
  FRIEND_TEST(CanopenDriverTest, test_init);
};

class CanopenDriverTest : public testing::Test
{
public:
  std::shared_ptr<MockCanopenMaster> canopen_driver;
  std::shared_ptr<MockNodeCanopenMaster> node_canopen_driver;
  void SetUp() override
  {
    RCLCPP_INFO(rclcpp::get_logger("CanopenDriverTest"), "SetUp");
    rclcpp::init(0, nullptr);
    canopen_driver = std::make_shared<MockCanopenMaster>();
    node_canopen_driver = std::make_shared<MockNodeCanopenMaster>();
    canopen_driver->node_canopen_driver_ =
      std::static_pointer_cast<ros2_canopen::node_interfaces::NodeCanopenDriverInterface>(
        node_canopen_driver);
  }

  void TearDown() override
  {
    RCLCPP_INFO(rclcpp::get_logger("CanopenDriverTest"), "TearDown");
    rclcpp::shutdown();
  }
};

TEST_F(CanopenDriverTest, test_init)
{
  EXPECT_CALL(*node_canopen_driver, init()).Times(1);
  canopen_driver->init();
}

TEST_F(CanopenDriverTest, test_set_master)
{
  std::shared_ptr<lely::ev::Executor> exec;
  std::shared_ptr<lely::canopen::AsyncMaster> master;

  EXPECT_CALL(*node_canopen_driver, set_master(_, _)).Times(1);
  EXPECT_NO_THROW(canopen_driver->set_master(exec, master));
}

TEST_F(CanopenDriverTest, test_get_node_base_interface)
{
  auto base_iface = canopen_driver->get_node_base_interface();
  EXPECT_TRUE(base_iface);
}

TEST_F(CanopenDriverTest, test_shutdown)
{
  EXPECT_CALL(*node_canopen_driver, shutdown());
  EXPECT_NO_THROW(canopen_driver->shutdown());
}

TEST_F(CanopenDriverTest, test_is_lifecycle) { EXPECT_TRUE(canopen_driver->is_lifecycle()); }

TEST_F(CanopenDriverTest, test_get_node_canopen_driver_interface)
{
  EXPECT_TRUE(canopen_driver->get_node_canopen_driver_interface() == node_canopen_driver);
}

TEST_F(CanopenDriverTest, test_on_configure)
{
  rclcpp_lifecycle::State state;
  EXPECT_CALL(*node_canopen_driver, configure());
  EXPECT_CALL(*node_canopen_driver, demand_set_master());
  canopen_driver->on_configure(state);
}

TEST_F(CanopenDriverTest, test_on_activate)
{
  rclcpp_lifecycle::State state;
  EXPECT_CALL(*node_canopen_driver, activate());
  canopen_driver->on_activate(state);
}

TEST_F(CanopenDriverTest, test_on_deactivate)
{
  rclcpp_lifecycle::State state;
  EXPECT_CALL(*node_canopen_driver, deactivate());
  canopen_driver->on_deactivate(state);
}

TEST_F(CanopenDriverTest, test_on_cleanup)
{
  rclcpp_lifecycle::State state;
  EXPECT_CALL(*node_canopen_driver, cleanup());
  canopen_driver->on_cleanup(state);
}

TEST_F(CanopenDriverTest, test_on_shutdown)
{
  rclcpp_lifecycle::State state;
  EXPECT_CALL(*node_canopen_driver, shutdown());
  canopen_driver->on_shutdown(state);
}
