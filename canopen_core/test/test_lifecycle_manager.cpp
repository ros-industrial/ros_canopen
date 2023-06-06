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
#include "canopen_core/lifecycle_manager.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
using namespace std::chrono_literals;
using namespace ros2_canopen;
using namespace testing;

class MockLifecycleManager : public LifecycleManager
{
public:
  FRIEND_TEST(LifecycleManagerTest, test_constructor);
  FRIEND_TEST(LifecycleManagerTest, test_init);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_master);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_master_false_initial_state);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_master_failed_activate);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_master_failed_configure);
  FRIEND_TEST(LifecycleManagerTest, test_bring_down_master);
  FRIEND_TEST(LifecycleManagerTest, test_bring_down_master_false_state);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_driver);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_driver_false_state_master);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_driver_false_state_driver);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_driver_fail_configure);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_driver_fail_activate);
  FRIEND_TEST(LifecycleManagerTest, test_bring_down_driver);
  FRIEND_TEST(LifecycleManagerTest, test_bring_down_driver_failed);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_all);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_all_fail_master);
  FRIEND_TEST(LifecycleManagerTest, test_bring_up_all_fail_driver);
  FRIEND_TEST(LifecycleManagerTest, test_bring_down_all);
  FRIEND_TEST(LifecycleManagerTest, test_bring_down_all_fail_master);
  FRIEND_TEST(LifecycleManagerTest, test_bring_down_all_fail_driver);
  FRIEND_TEST(LifecycleManagerTest, test_on_configure);
  FRIEND_TEST(LifecycleManagerTest, test_on_configure_fail);
  FRIEND_TEST(LifecycleManagerTest, test_on_activate);
  FRIEND_TEST(LifecycleManagerTest, test_on_activate_fail);
  FRIEND_TEST(LifecycleManagerTest, test_on_deactivate);
  FRIEND_TEST(LifecycleManagerTest, test_on_deactivate_fail);
  FRIEND_TEST(LifecycleManagerTest, test_on_cleanup);
  FRIEND_TEST(LifecycleManagerTest, test_on_shutdown);

  MockLifecycleManager() : LifecycleManager(rclcpp::NodeOptions()) {}
  MOCK_METHOD(unsigned int, get_state, (uint8_t node_id, std::chrono::seconds time_out));
  MOCK_METHOD(
    bool, change_state, (uint8_t node_id, uint8_t transition, std::chrono::seconds time_out));
  MOCK_METHOD(bool, bring_up_master, (), (override));
  MOCK_METHOD(bool, bring_down_master, (), (override));
  MOCK_METHOD(bool, bring_up_driver, (std::string device_name), (override));
  MOCK_METHOD(bool, bring_down_driver, (std::string device_name), (override));
  MOCK_METHOD(bool, load_from_config, (), (override));
  MOCK_METHOD(bool, bring_up_all, (), (override));
  MOCK_METHOD(bool, bring_down_all, (), (override));

  void DelegateToBase()
  {
    ON_CALL(*this, get_state)
      .WillByDefault(
        [this](uint8_t node_id, std::chrono::seconds time_out) -> unsigned int
        { return LifecycleManager::get_state(node_id, time_out); });
    ON_CALL(*this, change_state)
      .WillByDefault(
        [this](uint8_t node_id, uint8_t transition, std::chrono::seconds time_out) -> unsigned int
        { return LifecycleManager::change_state(node_id, transition, time_out); });
    ON_CALL(*this, bring_up_master)
      .WillByDefault([this]() -> bool { return LifecycleManager::bring_up_master(); });
    ON_CALL(*this, bring_down_master)
      .WillByDefault([this]() -> bool { return LifecycleManager::bring_down_master(); });
    ON_CALL(*this, bring_up_driver)
      .WillByDefault(
        [this](std::string device_name) -> bool
        { return LifecycleManager::bring_up_driver(device_name); });
    ON_CALL(*this, bring_down_driver)
      .WillByDefault(
        [this](std::string device_name) -> bool
        { return LifecycleManager::bring_down_driver(device_name); });
    ON_CALL(*this, load_from_config)
      .WillByDefault([this]() -> bool { return LifecycleManager::load_from_config(); });
    ON_CALL(*this, bring_up_all)
      .WillByDefault([this]() -> bool { return LifecycleManager::bring_up_all(); });
    ON_CALL(*this, bring_down_all)
      .WillByDefault([this]() -> bool { return LifecycleManager::bring_down_all(); });
  }
};

class LifecycleManagerTest : public testing::Test
{
public:
  std::shared_ptr<MockLifecycleManager> lifecycle_manager;
  void SetUp() override
  {
    RCLCPP_INFO(rclcpp::get_logger("CanopenMasterTest"), "SetUp");
    rclcpp::init(0, nullptr);
    lifecycle_manager = std::make_shared<MockLifecycleManager>();
    lifecycle_manager->DelegateToBase();
  }

  void TearDown() override
  {
    RCLCPP_INFO(rclcpp::get_logger("CanopenMasterTest"), "TearDown");
    rclcpp::shutdown();
  }

  void SetUpConfigurationManager()
  {
    std::string file_name("bus_configs/good_master_and_two_driver.yml");
    auto cmanager = std::make_shared<ConfigurationManager>(file_name);
    cmanager->init_config();
    lifecycle_manager->init(cmanager);
    EXPECT_CALL(*lifecycle_manager, load_from_config());
    lifecycle_manager->load_from_config();
  }
};

TEST_F(LifecycleManagerTest, test_constructor)
{
  EXPECT_TRUE(lifecycle_manager->has_parameter("container_name"));
  EXPECT_TRUE(lifecycle_manager->cbg_clients);
}

TEST_F(LifecycleManagerTest, test_init)
{
  std::string file_name("bus_configs/good_master_and_two_driver.yml");
  auto cmanager = std::make_shared<ConfigurationManager>(file_name);
  cmanager->init_config();
  lifecycle_manager->init(cmanager);
  EXPECT_TRUE(lifecycle_manager->config_);
}

TEST_F(LifecycleManagerTest, test_bring_up_master)
{
  EXPECT_CALL(*lifecycle_manager, bring_up_master());
  EXPECT_CALL(*lifecycle_manager, get_state(_, _)).Times(1).WillOnce(Return(1U));
  EXPECT_CALL(*lifecycle_manager, change_state(_, _, _)).Times(2).WillRepeatedly(Return(true));
  EXPECT_TRUE(lifecycle_manager->bring_up_master());
}

TEST_F(LifecycleManagerTest, test_bring_up_master_false_initial_state)
{
  EXPECT_CALL(*lifecycle_manager, bring_up_master());
  EXPECT_CALL(*lifecycle_manager, get_state(_, _)).Times(1).WillOnce(Return(0U));
  EXPECT_FALSE(lifecycle_manager->bring_up_master());
}

TEST_F(LifecycleManagerTest, test_bring_up_master_failed_configure)
{
  EXPECT_CALL(*lifecycle_manager, bring_up_master());
  EXPECT_CALL(*lifecycle_manager, get_state(_, _)).Times(1).WillOnce(Return(1U));
  EXPECT_CALL(*lifecycle_manager, change_state(_, _, _)).Times(1).WillOnce(Return(false));
  EXPECT_FALSE(lifecycle_manager->bring_up_master());
}

TEST_F(LifecycleManagerTest, test_bring_up_master_failed_activate)
{
  EXPECT_CALL(*lifecycle_manager, bring_up_master());
  EXPECT_CALL(*lifecycle_manager, get_state(_, _)).Times(1).WillOnce(Return(1U));
  EXPECT_CALL(*lifecycle_manager, change_state(_, _, _))
    .Times(2)
    .WillOnce(Return(true))
    .WillOnce(Return(false));
  EXPECT_FALSE(lifecycle_manager->bring_up_master());
}

TEST_F(LifecycleManagerTest, test_bring_down_master)
{
  EXPECT_CALL(*lifecycle_manager, bring_down_master());
  EXPECT_CALL(*lifecycle_manager, change_state(_, _, _)).Times(2).WillRepeatedly(Return(true));
  EXPECT_CALL(*lifecycle_manager, get_state(_, _))
    .Times(1)
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED));
  EXPECT_TRUE(lifecycle_manager->bring_down_master());
}

TEST_F(LifecycleManagerTest, test_bring_down_master_false_state)
{
  EXPECT_CALL(*lifecycle_manager, bring_down_master());
  EXPECT_CALL(*lifecycle_manager, change_state(_, _, _)).Times(2).WillRepeatedly(Return(true));
  EXPECT_CALL(*lifecycle_manager, get_state(_, _))
    .Times(1)
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
  EXPECT_FALSE(lifecycle_manager->bring_down_master());
}

TEST_F(LifecycleManagerTest, test_bring_up_driver)
{
  SetUpConfigurationManager();
  EXPECT_CALL(*lifecycle_manager, bring_up_driver(_));
  EXPECT_CALL(*lifecycle_manager, get_state(_, _))
    .Times(2)
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE))
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED));
  EXPECT_CALL(*lifecycle_manager, change_state(_, _, _)).Times(2).WillRepeatedly(Return(true));
  EXPECT_TRUE(lifecycle_manager->bring_up_driver("proxy_device_2"));
}

TEST_F(LifecycleManagerTest, test_bring_up_driver_false_state_master)
{
  SetUpConfigurationManager();

  EXPECT_CALL(*lifecycle_manager, bring_up_driver(_));
  EXPECT_CALL(*lifecycle_manager, get_state(_, _))
    .Times(1)
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED));
  EXPECT_FALSE(lifecycle_manager->bring_up_driver("proxy_device_2"));
}

TEST_F(LifecycleManagerTest, test_bring_up_driver_false_state_driver)
{
  SetUpConfigurationManager();

  EXPECT_CALL(*lifecycle_manager, bring_up_driver(_));
  EXPECT_CALL(*lifecycle_manager, get_state(_, _))
    .Times(2)
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE))
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
  EXPECT_FALSE(lifecycle_manager->bring_up_driver("proxy_device_2"));
}

TEST_F(LifecycleManagerTest, test_bring_up_driver_fail_configure)
{
  SetUpConfigurationManager();

  EXPECT_CALL(*lifecycle_manager, bring_up_driver(_));
  EXPECT_CALL(*lifecycle_manager, get_state(_, _))
    .Times(2)
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE))
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED));
  EXPECT_CALL(*lifecycle_manager, change_state(_, _, _)).Times(1).WillOnce(Return(false));
  EXPECT_FALSE(lifecycle_manager->bring_up_driver("proxy_device_2"));
}

TEST_F(LifecycleManagerTest, test_bring_up_driver_fail_activate)
{
  SetUpConfigurationManager();

  EXPECT_CALL(*lifecycle_manager, bring_up_driver(_));
  EXPECT_CALL(*lifecycle_manager, get_state(_, _))
    .Times(2)
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE))
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED));
  EXPECT_CALL(*lifecycle_manager, change_state(_, _, _))
    .Times(2)
    .WillOnce(Return(true))
    .WillOnce(Return(false));
  EXPECT_FALSE(lifecycle_manager->bring_up_driver("proxy_device_2"));
}

TEST_F(LifecycleManagerTest, test_bring_down_driver)
{
  SetUpConfigurationManager();

  EXPECT_CALL(*lifecycle_manager, bring_down_driver(_));
  EXPECT_CALL(*lifecycle_manager, change_state(_, _, _))
    .Times(2)
    .WillOnce(Return(true))
    .WillOnce(Return(true));
  EXPECT_CALL(*lifecycle_manager, get_state(_, _))
    .Times(1)
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED));
  EXPECT_TRUE(lifecycle_manager->bring_down_driver("proxy_device_2"));
}

TEST_F(LifecycleManagerTest, test_bring_down_driver_failed)
{
  SetUpConfigurationManager();

  EXPECT_CALL(*lifecycle_manager, bring_down_driver(_));
  EXPECT_CALL(*lifecycle_manager, change_state(_, _, _))
    .Times(2)
    .WillOnce(Return(true))
    .WillOnce(Return(true));
  EXPECT_CALL(*lifecycle_manager, get_state(_, _))
    .Times(1)
    .WillOnce(Return(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
  EXPECT_FALSE(lifecycle_manager->bring_down_driver("proxy_device_2"));
}

TEST_F(LifecycleManagerTest, test_bring_up_all)
{
  SetUpConfigurationManager();

  EXPECT_CALL(*lifecycle_manager, bring_up_all);
  EXPECT_CALL(*lifecycle_manager, bring_up_master()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*lifecycle_manager, bring_up_driver(_)).Times(2).WillRepeatedly(Return(true));
  EXPECT_TRUE(lifecycle_manager->bring_up_all());
}

TEST_F(LifecycleManagerTest, test_bring_up_all_fail_master)
{
  SetUpConfigurationManager();

  EXPECT_CALL(*lifecycle_manager, bring_up_all);
  EXPECT_CALL(*lifecycle_manager, bring_up_master()).Times(1).WillOnce(Return(false));
  EXPECT_FALSE(lifecycle_manager->bring_up_all());
}

TEST_F(LifecycleManagerTest, test_bring_up_all_fail_driver)
{
  SetUpConfigurationManager();

  EXPECT_CALL(*lifecycle_manager, bring_up_all);
  EXPECT_CALL(*lifecycle_manager, bring_up_master()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*lifecycle_manager, bring_up_driver(_)).Times(1).WillRepeatedly(Return(false));
  EXPECT_FALSE(lifecycle_manager->bring_up_all());
}

TEST_F(LifecycleManagerTest, test_bring_down_all)
{
  SetUpConfigurationManager();
  EXPECT_CALL(*lifecycle_manager, bring_down_all);
  EXPECT_CALL(*lifecycle_manager, bring_down_driver(_)).Times(2).WillRepeatedly(Return(true));
  EXPECT_CALL(*lifecycle_manager, bring_down_master()).Times(1).WillOnce(Return(true));
  EXPECT_TRUE(lifecycle_manager->bring_down_all());
}

TEST_F(LifecycleManagerTest, test_bring_down_all_fail_driver)
{
  SetUpConfigurationManager();
  EXPECT_CALL(*lifecycle_manager, bring_down_all);
  EXPECT_CALL(*lifecycle_manager, bring_down_driver(_)).Times(1).WillRepeatedly(Return(false));
  EXPECT_FALSE(lifecycle_manager->bring_down_all());
}

TEST_F(LifecycleManagerTest, test_bring_down_all_fail_master)
{
  SetUpConfigurationManager();
  EXPECT_CALL(*lifecycle_manager, bring_down_all);
  EXPECT_CALL(*lifecycle_manager, bring_down_driver(_)).Times(2).WillRepeatedly(Return(true));
  EXPECT_CALL(*lifecycle_manager, bring_down_master()).Times(1).WillOnce(Return(false));
  EXPECT_FALSE(lifecycle_manager->bring_down_all());
}

TEST_F(LifecycleManagerTest, test_on_configure)
{
  rclcpp_lifecycle::State state;
  EXPECT_CALL(*lifecycle_manager, load_from_config()).Times(1).WillOnce(Return(true));
  EXPECT_EQ(
    lifecycle_manager->on_configure(state),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(LifecycleManagerTest, test_on_configure_fail)
{
  rclcpp_lifecycle::State state;
  EXPECT_CALL(*lifecycle_manager, load_from_config()).Times(1).WillOnce(Return(false));
  EXPECT_EQ(
    lifecycle_manager->on_configure(state),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR);
}

TEST_F(LifecycleManagerTest, test_on_activate)
{
  rclcpp_lifecycle::State state;
  EXPECT_CALL(*lifecycle_manager, bring_up_all()).Times(1).WillOnce(Return(true));
  EXPECT_EQ(
    lifecycle_manager->on_activate(state),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(LifecycleManagerTest, test_on_activate_fail)
{
  rclcpp_lifecycle::State state;
  EXPECT_CALL(*lifecycle_manager, bring_up_all()).Times(1).WillOnce(Return(false));
  EXPECT_EQ(
    lifecycle_manager->on_activate(state),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR);
}

TEST_F(LifecycleManagerTest, test_on_deactivate)
{
  rclcpp_lifecycle::State state;
  EXPECT_CALL(*lifecycle_manager, bring_down_all()).Times(1).WillOnce(Return(true));
  EXPECT_EQ(
    lifecycle_manager->on_deactivate(state),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(LifecycleManagerTest, test_on_deactivate_fail)
{
  rclcpp_lifecycle::State state;
  EXPECT_CALL(*lifecycle_manager, bring_down_all()).Times(1).WillOnce(Return(false));
  EXPECT_EQ(
    lifecycle_manager->on_deactivate(state),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR);
}

TEST_F(LifecycleManagerTest, test_on_cleanup)
{
  rclcpp_lifecycle::State state;
  EXPECT_EQ(
    lifecycle_manager->on_cleanup(state),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(LifecycleManagerTest, test_on_shutdown)
{
  rclcpp_lifecycle::State state;
  EXPECT_EQ(
    lifecycle_manager->on_shutdown(state),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}
