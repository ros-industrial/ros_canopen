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
#include "canopen_core/device_container.hpp"
#include "canopen_core/device_container_error.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
using namespace std::chrono_literals;
using namespace ros2_canopen;
using namespace testing;

class MockNodeFactory : public rclcpp_components::NodeFactory
{
public:
  MOCK_METHOD(
    rclcpp_components::NodeInstanceWrapper, create_node_instance,
    (const rclcpp::NodeOptions & options), (override));
};

class FakeDeviceContainer
{
public:
  virtual std::vector<rclcpp_components::ComponentManager::ComponentResource>
  get_component_resources(
    const std::string & package_name,
    const std::string & resource_index = "rclcpp_components") const
  {
    // RCLCPP_INFO(this->get_logger(), "get_component_resources");
    std::vector<rclcpp_components::ComponentManager::ComponentResource> components;
    components.push_back(rclcpp_components::ComponentManager::ComponentResource(
      "ros2_canopen::CanopenMaster", "canopen_core"));
    components.push_back(rclcpp_components::ComponentManager::ComponentResource(
      "ros2_canopen::LifecycleCanopenMaster", "canopen_core"));
    components.push_back(rclcpp_components::ComponentManager::ComponentResource(
      "ros2_canopen::CanopenDriver", "canopen_core"));
    components.push_back(rclcpp_components::ComponentManager::ComponentResource(
      "ros2_canopen::LifecycleCanopenDriver", "canopen_core"));
    // get_component_resources_mock(true);
    return components;
  }
};
using namespace std::placeholders;
class MockDeviceContainer : public DeviceContainer
{
  FakeDeviceContainer fake_device_container;

public:
  FRIEND_TEST(DeviceContainerTest, test_load_component_master);
  FRIEND_TEST(DeviceContainerTest, test_load_component_driver_non_lifecycle);
  FRIEND_TEST(DeviceContainerTest, test_load_component_driver_lifecycle);
  FRIEND_TEST(DeviceContainerTest, test_get_registered_drivers);
  FRIEND_TEST(DeviceContainerTest, test_shutdown);
  FRIEND_TEST(DeviceContainerTest, test_get_ids_of_drivers_with_type);
  FRIEND_TEST(DeviceContainerTest, test_get_driver_type);
  FRIEND_TEST(DeviceContainerTest, test_on_list_nodes);
  FRIEND_TEST(DeviceContainerTest, test_load_master_good);
  FRIEND_TEST(DeviceContainerTest, test_load_master_load_component_fail);
  FRIEND_TEST(DeviceContainerTest, test_load_master_bad_no_id);
  FRIEND_TEST(DeviceContainerTest, test_load_master_bad_no_driver);
  FRIEND_TEST(DeviceContainerTest, test_load_master_bad_no_package);
  FRIEND_TEST(DeviceContainerTest, test_load_driver_good);
  FRIEND_TEST(DeviceContainerTest, test_load_driver_load_component_fail);
  FRIEND_TEST(DeviceContainerTest, test_load_driver_bad_no_id);
  FRIEND_TEST(DeviceContainerTest, test_load_driver_bad_no_package);
  FRIEND_TEST(DeviceContainerTest, test_load_driver_bad_no_driver);
  FRIEND_TEST(DeviceContainerTest, test_load_manager_no_lifecycle);
  FRIEND_TEST(DeviceContainerTest, test_load_manager_lifecycle);
  FRIEND_TEST(DeviceContainerTest, test_configure_good);

  MockDeviceContainer(
    std::weak_ptr<rclcpp::Executor> executor =
      std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>())
  : DeviceContainer(executor), fake_device_container()
  {
  }
  MOCK_METHOD(
    bool, load_component,
    (std::string & package_name, std::string & driver_name, uint16_t node_id,
     std::string & node_name, std::vector<rclcpp::Parameter> & params),
    (override));
  MOCK_METHOD(void, configure, (), (override));
  MOCK_METHOD(bool, load_master, (), (override));
  MOCK_METHOD(bool, load_drivers, (), (override));
  MOCK_METHOD(bool, load_manager, (), (override));
  MOCK_METHOD(
    std::shared_ptr<rclcpp_components::NodeFactory>, create_component_factory,
    (const ComponentResource & resource), (override));
  MOCK_METHOD(
    std::vector<ComponentResource>, get_component_resources,
    (const std::string & package_name, const std::string & resource_index), (const, override));
  MOCK_METHOD(
    void, add_node_to_executor,
    (rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_interface), (override));

  void DelegateToFake()
  {
    ON_CALL(*this, load_component)
      .WillByDefault(
        [this](
          std::string & package_name, std::string & driver_name, uint16_t node_id,
          std::string & node_name, std::vector<rclcpp::Parameter> & params) -> bool {
          return DeviceContainer::load_component(
            package_name, driver_name, node_id, node_name, params);
        });
    ON_CALL(*this, load_master)
      .WillByDefault([this]() -> bool { return DeviceContainer::load_master(); });
    ON_CALL(*this, load_drivers)
      .WillByDefault([this]() -> bool { return DeviceContainer::load_drivers(); });
    ON_CALL(*this, load_manager)
      .WillByDefault([this]() -> bool { return DeviceContainer::load_manager(); });
    ON_CALL(*this, configure).WillByDefault([this]() { return DeviceContainer::configure(); });
    ON_CALL(*this, get_component_resources)
      .WillByDefault(
        [this](const std::string & package_name, const std::string & resource_index)
          -> std::vector<ComponentResource>
        { return fake_device_container.get_component_resources(package_name, resource_index); });
    ON_CALL(*this, add_node_to_executor)
      .WillByDefault([this](rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_interface)
                     { DeviceContainer::add_node_to_executor(node_interface); });
  }
};

class MockCanopenDriver : public CanopenDriverInterface, public rclcpp::Node
{
public:
  explicit MockCanopenDriver(
    std::string name = "mock_canopen_driver",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : rclcpp::Node(name, node_options)
  {
  }
  MOCK_METHOD(void, init, (), (override));
  MOCK_METHOD(
    void, set_master,
    (std::shared_ptr<lely::ev::Executor> exec, std::shared_ptr<lely::canopen::AsyncMaster> master),
    (override));
  // MOCK_METHOD(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr, get_node_base_interface, (),
  // (override));
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return rclcpp::Node::get_node_base_interface();
  }
  MOCK_METHOD(void, shutdown, (), (override));
  MOCK_METHOD(bool, is_lifecycle, (), (override));
  MOCK_METHOD(
    std::shared_ptr<ros2_canopen::node_interfaces::NodeCanopenDriverInterface>,
    get_node_canopen_driver_interface, (), (override));
};

class MockCanopenMaster : public CanopenMasterInterface, public rclcpp::Node
{
public:
  explicit MockCanopenMaster(
    std::string name = "mock_canopen_master",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : rclcpp::Node(name, node_options)
  {
  }
  MOCK_METHOD(void, init, (), (override));
  MOCK_METHOD(void, shutdown, (), (override));
  MOCK_METHOD(std::shared_ptr<lely::canopen::AsyncMaster>, get_master, (), (override));
  MOCK_METHOD(std::shared_ptr<lely::ev::Executor>, get_executor, (), (override));
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return rclcpp::Node::get_node_base_interface();
  }
  MOCK_METHOD(bool, is_lifecycle, (), (override));
};

class DeviceContainerTest : public testing::Test
{
public:
  std::shared_ptr<MockDeviceContainer> device_container;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;
  std::thread spinThread;
  void SetUp() override
  {
    RCLCPP_INFO(rclcpp::get_logger("DeviceContainerTest"), "SetUp");
    rclcpp::init(0, nullptr);
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    device_container = std::make_shared<MockDeviceContainer>(exec);
    device_container->DelegateToFake();

    exec->add_node(device_container);
    spinThread = std::thread([this]() { exec->spin(); });
  }

  void TearDown() override
  {
    RCLCPP_INFO(rclcpp::get_logger("DeviceContainerTest"), "Teardown");
    exec->cancel();
    spinThread.join();
    RCLCPP_INFO(rclcpp::get_logger("DeviceContainerTest"), "Executor joined.");
    exec->remove_node(device_container);
    RCLCPP_INFO(rclcpp::get_logger("DeviceContainerTest"), "Node removed.");
    rclcpp::shutdown();
  }
};

TEST_F(DeviceContainerTest, test_name_and_parameters_declared)
{
  EXPECT_TRUE(std::string("device_container").compare(device_container->get_name()) == 0);
  EXPECT_TRUE(device_container->has_parameter("can_interface_name"));
  EXPECT_TRUE(device_container->has_parameter("master_config"));
  EXPECT_TRUE(device_container->has_parameter("bus_config"));
  EXPECT_TRUE(device_container->has_parameter("master_bin"));
}

TEST_F(DeviceContainerTest, test_services_declared)
{
  auto services = device_container->get_service_names_and_types_by_node(
    device_container->get_name(), device_container->get_namespace());
  bool found_init_driver = false;
  bool found_unload_node = false;
  bool found_load_node = false;
  for (auto it = services.begin(); it != services.end(); ++it)
  {
    if (it->first.compare("/device_container/init_driver") == 0)
    {
      found_init_driver = true;
    }
    if (it->first.compare("/device_container/_container/unload_node") == 0)
    {
      found_unload_node = true;
    }
    if (it->first.compare("/device_container/_container/load_node") == 0)
    {
      found_load_node = true;
    }
  }
  EXPECT_TRUE(found_init_driver);
  EXPECT_FALSE(found_unload_node);
  EXPECT_FALSE(found_load_node);
}

TEST_F(DeviceContainerTest, test_load_component_master)
{
  std::vector<rclcpp::Parameter> params;
  std::string package("canopen_core");
  std::string driver_name("ros2_canopen::CanopenMaster");
  std::string node_name("master");
  uint16_t node_id = 1;
  auto node = std::make_shared<MockCanopenMaster>();
  auto node_factory = std::make_shared<MockNodeFactory>();
  auto node_instance = rclcpp_components::NodeInstanceWrapper(
    std::static_pointer_cast<void>(node),
    std::bind(&MockCanopenMaster::get_node_base_interface, node));
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_)).Times(1).WillOnce(Return(node_instance));
  device_container->load_component(package, driver_name, node_id, node_name, params);

  // Check that can_master_ is assigned.
  EXPECT_TRUE(((long)device_container->can_master_.get() == (long)node.get()));
  // Check that master is not stored in driver map.
  EXPECT_FALSE((long)device_container->registered_drivers_[node_id].get() == (long)node.get());
}

TEST_F(DeviceContainerTest, test_load_component_driver_non_lifecycle)
{
  // Lifecycle operation false
  device_container->lifecycle_operation_ = false;
  std::vector<rclcpp::Parameter> params;
  std::string package("canopen_core");
  std::string driver_name("ros2_canopen::CanopenDriver");
  std::string node_name("driver");

  uint16_t node_id = 1;
  auto node = std::make_shared<MockCanopenDriver>();
  auto node_factory = std::make_shared<MockNodeFactory>();
  auto node_instance = rclcpp_components::NodeInstanceWrapper(
    std::static_pointer_cast<void>(node),
    std::bind(&MockCanopenDriver::get_node_base_interface, node));

  // Lifecycle component leads to throw
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_)).Times(1).WillOnce(Return(node_instance));
  EXPECT_CALL(*node, is_lifecycle()).Times(1).WillOnce(Return(true));
  EXPECT_THROW(
    device_container->load_component(package, driver_name, node_id, node_name, params),
    DeviceContainerException);

  // Non lifecycle component leads to normal behaviour
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_)).Times(1).WillOnce(Return(node_instance));
  EXPECT_CALL(*node, is_lifecycle()).Times(1).WillOnce(Return(false));
  device_container->load_component(package, driver_name, node_id, node_name, params);

  // Check that can_master_ is not assigned.
  EXPECT_FALSE(((long)device_container->can_master_.get() == (long)node.get()));
  // Check that driver is stored in driver map.
  EXPECT_TRUE((long)device_container->registered_drivers_[node_id].get() == (long)node.get());
}

TEST_F(DeviceContainerTest, test_load_component_driver_lifecycle)
{
  // Lifecycle operation true
  device_container->lifecycle_operation_ = true;
  std::vector<rclcpp::Parameter> params;
  std::string package("canopen_core");
  std::string driver_name("ros2_canopen::CanopenDriver");
  std::string node_name("driver");
  uint16_t node_id = 1;
  auto node = std::make_shared<MockCanopenDriver>();
  auto node_factory = std::make_shared<MockNodeFactory>();
  auto node_instance = rclcpp_components::NodeInstanceWrapper(
    std::static_pointer_cast<void>(node),
    std::bind(&MockCanopenDriver::get_node_base_interface, node));

  // Non lifecycle component leads to throw
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_)).Times(1).WillOnce(Return(node_instance));
  EXPECT_CALL(*node, is_lifecycle()).Times(1).WillOnce(Return(false));
  EXPECT_THROW(
    device_container->load_component(package, driver_name, node_id, node_name, params),
    DeviceContainerException);

  // Lifecycle component leads to normal behaviour
  node_id = 2;
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_)).Times(1).WillOnce(Return(node_instance));
  EXPECT_CALL(*node, is_lifecycle()).Times(1).WillOnce(Return(true));
  device_container->load_component(package, driver_name, node_id, node_name, params);

  // Check that can_master_ is not assigned.
  EXPECT_FALSE(((long)device_container->can_master_.get() == (long)node.get()));
  // Check that driver is stored in driver map.
  EXPECT_TRUE((long)device_container->registered_drivers_[node_id].get() == (long)node.get());
}

TEST_F(DeviceContainerTest, test_get_registered_drivers)
{
  // Lifecycle operation true
  device_container->lifecycle_operation_ = false;
  std::vector<rclcpp::Parameter> params;
  std::string package("canopen_core");
  std::string driver_name("ros2_canopen::CanopenDriver");
  std::string node_name("driver");
  uint16_t node_id = 1;
  auto node = std::make_shared<MockCanopenDriver>();
  auto node_factory = std::make_shared<MockNodeFactory>();
  auto node_instance = rclcpp_components::NodeInstanceWrapper(
    std::static_pointer_cast<void>(node),
    std::bind(&MockCanopenDriver::get_node_base_interface, node));

  // Should be zero at start
  EXPECT_EQ(device_container->get_registered_drivers().size(), 0U);

  // Load component and expect that size of registered drivers changes to 1
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_)).Times(1).WillOnce(Return(node_instance));
  EXPECT_CALL(*node, is_lifecycle()).Times(1).WillOnce(Return(false));
  device_container->load_component(package, driver_name, node_id, node_name, params);
  EXPECT_EQ(device_container->get_registered_drivers().size(), 1U);

  // Load component and expect that size of registered drivers changes to 2
  node_id = 2;
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_)).Times(1).WillOnce(Return(node_instance));
  EXPECT_CALL(*node, is_lifecycle()).Times(1).WillOnce(Return(false));
  device_container->load_component(package, driver_name, node_id, node_name, params);
  EXPECT_EQ(device_container->get_registered_drivers().size(), 2U);
}

TEST_F(DeviceContainerTest, test_count_drivers)
{
  std::vector<rclcpp::Parameter> params;
  std::string package("canopen_core");
  std::string driver_name("ros2_canopen::CanopenDriver");
  std::string node_name("driver1");
  uint16_t node_id = 1;
  auto node = std::make_shared<MockCanopenDriver>();
  auto node_factory = std::make_shared<MockNodeFactory>();
  auto node_instance = rclcpp_components::NodeInstanceWrapper(
    std::static_pointer_cast<void>(node),
    std::bind(&MockCanopenDriver::get_node_base_interface, node));

  // Should be zero at start
  EXPECT_EQ(device_container->count_drivers(), 0U);

  // Load component and expect that size of registered drivers changes to 1
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_)).Times(1).WillOnce(Return(node_instance));
  EXPECT_CALL(*node, is_lifecycle()).Times(1).WillOnce(Return(false));
  device_container->load_component(package, driver_name, node_id, node_name, params);
  EXPECT_EQ(device_container->count_drivers(), 1U);

  // Load component and expect that size of registered drivers changes to 2
  node_id = 2;
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_)).Times(1).WillOnce(Return(node_instance));
  EXPECT_CALL(*node, is_lifecycle()).Times(1).WillOnce(Return(false));
  device_container->load_component(package, driver_name, node_id, node_name, params);
  EXPECT_EQ(device_container->count_drivers(), 2U);
}

TEST_F(DeviceContainerTest, test_shutdown)
{
  // Lifecycle operation true
  std::vector<rclcpp::Parameter> params;
  std::string package("canopen_core");
  std::string driver_name("ros2_canopen::CanopenMaster");
  std::string node_name("master");
  uint16_t node_id = 1;
  auto master_node = std::make_shared<MockCanopenMaster>();
  auto driver_node = std::make_shared<MockCanopenDriver>();
  auto node_factory = std::make_shared<MockNodeFactory>();
  auto master_node_instance = rclcpp_components::NodeInstanceWrapper(
    std::static_pointer_cast<void>(master_node),
    std::bind(&MockCanopenMaster::get_node_base_interface, master_node));
  auto driver_node_instance = rclcpp_components::NodeInstanceWrapper(
    std::static_pointer_cast<void>(driver_node),
    std::bind(&MockCanopenDriver::get_node_base_interface, driver_node));

  // Load component and expect that size of registered drivers changes to 1
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_))
    .Times(1)
    .WillOnce(Return(master_node_instance));
  device_container->load_component(package, driver_name, node_id, node_name, params);

  driver_name = "ros2_canopen::CanopenDriver";
  node_name = "driver2";
  node_id = 2;
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_))
    .Times(1)
    .WillOnce(Return(driver_node_instance));
  EXPECT_CALL(*driver_node, is_lifecycle()).Times(1).WillOnce(Return(false));
  device_container->load_component(package, driver_name, node_id, node_name, params);

  node_id = 3;
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_))
    .Times(1)
    .WillOnce(Return(driver_node_instance));
  EXPECT_CALL(*driver_node, is_lifecycle()).Times(1).WillOnce(Return(false));
  device_container->load_component(package, driver_name, node_id, node_name, params);
  EXPECT_EQ(device_container->get_registered_drivers().size(), 2U);

  EXPECT_CALL(*driver_node, shutdown()).Times(2);
  EXPECT_CALL(*master_node, shutdown()).Times(1);
  device_container->shutdown();
}

TEST_F(DeviceContainerTest, test_get_ids_of_drivers_with_type)
{
  std::string file_name("bus_configs/good_master_and_two_driver.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();
  std::string driver_name("ros2_canopen::CanopenDriver");
  auto ids = device_container->get_ids_of_drivers_with_type(driver_name);
  EXPECT_EQ(ids.size(), 2U);
}

TEST_F(DeviceContainerTest, test_get_driver_type)
{
  std::string file_name("bus_configs/good_master_and_two_driver.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();
  uint16_t node_id = 2;
  auto driver_type = device_container->get_driver_type(node_id);
  EXPECT_EQ(driver_type.compare("ros2_canopen::CanopenDriver"), 0);
}

TEST_F(DeviceContainerTest, test_on_list_nodes)
{
  // Lifecycle operation true
  std::vector<rclcpp::Parameter> params;
  std::string package("canopen_core");
  std::string master_name("ros2_canopen::CanopenMaster");
  std::string driver_name("ros2_canopen::CanopenDriver");
  std::string master_node_name("master");
  std::string driver_node_name("driver");
  uint16_t node_id;
  auto master_node = std::make_shared<MockCanopenMaster>(master_node_name);
  auto driver_node = std::make_shared<MockCanopenDriver>(driver_node_name);
  device_container->lifecycle_manager_ = std::make_unique<LifecycleManager>(rclcpp::NodeOptions());

  auto node_factory = std::make_shared<MockNodeFactory>();
  auto master_node_instance = rclcpp_components::NodeInstanceWrapper(
    std::static_pointer_cast<void>(master_node),
    std::bind(&MockCanopenMaster::get_node_base_interface, master_node));
  auto driver_node_instance = rclcpp_components::NodeInstanceWrapper(
    std::static_pointer_cast<void>(driver_node),
    std::bind(&MockCanopenDriver::get_node_base_interface, driver_node));

  node_id = 1;
  device_container->can_master_id_ = node_id;
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_))
    .Times(1)
    .WillOnce(Return(master_node_instance));
  device_container->load_component(package, master_name, node_id, master_node_name, params);

  node_id = 2;
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _));
  EXPECT_CALL(*device_container, get_component_resources(_, _));
  EXPECT_CALL(*device_container, create_component_factory(_))
    .Times(1)
    .WillOnce(Return(node_factory));
  EXPECT_CALL(*node_factory, create_node_instance(_))
    .Times(1)
    .WillOnce(Return(driver_node_instance));
  EXPECT_CALL(*driver_node, is_lifecycle()).Times(1).WillOnce(Return(false));
  device_container->load_component(package, driver_name, node_id, driver_node_name, params);

  const std::shared_ptr<rmw_request_id_t> request_header(nullptr);
  auto request = std::make_shared<composition_interfaces::srv::ListNodes::Request>();
  auto response = std::make_shared<composition_interfaces::srv::ListNodes::Response>();

  device_container->on_list_nodes(request_header, request, response);

  for (size_t i = 0; i < response->full_node_names.size(); i++)
  {
    if (response->full_node_names[i].compare(std::string("/").append(master_node_name)) == 0)
    {
      EXPECT_EQ(response->unique_ids[i], 1U);
    }
    else if (response->full_node_names[i].compare(std::string("/").append(driver_node_name)) == 0)
    {
      EXPECT_EQ(response->unique_ids[i], 2U);
    }
    else if (response->full_node_names[i].compare(std::string("/").append(driver_node_name)) == 0)
    {
      EXPECT_EQ(response->unique_ids[i], 256U);
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("TEST"), response->full_node_names[i].c_str());
      FAIL();
    }
  }
}

TEST_F(DeviceContainerTest, test_load_master_good)
{
  std::string file_name("bus_configs/good_master.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();
  auto can_master = std::make_shared<MockCanopenMaster>("master");

  EXPECT_CALL(*device_container, load_master());
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _))
    .WillOnce(
      [this, can_master](
        std::string & package_name, std::string & driver_name, uint16_t node_id,
        std::string & node_name, std::vector<rclcpp::Parameter> & params) -> bool
      {
        device_container->can_master_ =
          std::static_pointer_cast<ros2_canopen::CanopenMasterInterface>(can_master);
        return true;
      });
  EXPECT_CALL(*device_container, add_node_to_executor(_));
  EXPECT_CALL(*can_master, init());
  EXPECT_CALL(*can_master, is_lifecycle());
  EXPECT_TRUE(device_container->load_master());
}

TEST_F(DeviceContainerTest, test_load_master_load_component_fail)
{
  std::string file_name("bus_configs/good_master.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();
  auto can_master = std::make_shared<MockCanopenMaster>("master");

  EXPECT_CALL(*device_container, load_master());
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _)).WillOnce(Return(false));
  EXPECT_FALSE(device_container->load_master());
}

TEST_F(DeviceContainerTest, test_load_master_bad_no_driver)
{
  std::string file_name("bus_configs/bad_master_no_driver.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();
  auto can_master = std::make_shared<MockCanopenMaster>("master");

  EXPECT_CALL(*device_container, load_master());
  EXPECT_FALSE(device_container->load_master());
}

TEST_F(DeviceContainerTest, test_load_master_bad_no_id)
{
  std::string file_name("bus_configs/bad_master_no_id.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();
  auto can_master = std::make_shared<MockCanopenMaster>("master");

  EXPECT_CALL(*device_container, load_master());
  EXPECT_FALSE(device_container->load_master());
}

TEST_F(DeviceContainerTest, test_load_master_bad_no_package)
{
  std::string file_name("bus_configs/bad_master_no_package.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();
  auto can_master = std::make_shared<MockCanopenMaster>("master");

  EXPECT_CALL(*device_container, load_master());
  EXPECT_FALSE(device_container->load_master());
}

TEST_F(DeviceContainerTest, test_load_driver_good)
{
  std::string file_name("bus_configs/good_driver.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();
  auto driver = std::make_shared<MockCanopenDriver>("driver");

  EXPECT_CALL(*device_container, load_drivers());
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _))
    .WillOnce(
      [this, driver](
        std::string & package_name, std::string & driver_name, uint16_t node_id,
        std::string & node_name, std::vector<rclcpp::Parameter> & params) -> bool
      {
        device_container->registered_drivers_[2] =
          std::static_pointer_cast<ros2_canopen::CanopenDriverInterface>(driver);
        return true;
      });
  EXPECT_CALL(*device_container, add_node_to_executor(_));
  EXPECT_CALL(*driver, init());
  EXPECT_TRUE(device_container->load_drivers());
}

TEST_F(DeviceContainerTest, test_load_driver_load_component_fail)
{
  std::string file_name("bus_configs/good_driver.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();
  auto driver = std::make_shared<MockCanopenDriver>("driver");

  EXPECT_CALL(*device_container, load_drivers());
  EXPECT_CALL(*device_container, load_component(_, _, _, _, _)).WillOnce(Return(false));
  EXPECT_FALSE(device_container->load_drivers());
}

TEST_F(DeviceContainerTest, test_load_driver_bad_no_id)
{
  std::string file_name("bus_configs/bad_driver_no_id.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();

  EXPECT_CALL(*device_container, load_drivers());
  EXPECT_FALSE(device_container->load_drivers());
}

TEST_F(DeviceContainerTest, test_load_driver_bad_no_driver)
{
  std::string file_name("bus_configs/bad_driver_no_driver.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();

  EXPECT_CALL(*device_container, load_drivers());
  EXPECT_FALSE(device_container->load_drivers());
}

TEST_F(DeviceContainerTest, test_load_driver_bad_no_package)
{
  std::string file_name("bus_configs/bad_driver_no_package.yml");
  device_container->config_ = std::make_shared<ConfigurationManager>(file_name);
  device_container->config_->init_config();

  EXPECT_CALL(*device_container, load_drivers());
  EXPECT_FALSE(device_container->load_drivers());
}

TEST_F(DeviceContainerTest, test_load_manager_no_lifecycle)
{
  device_container->lifecycle_operation_ = false;
  EXPECT_CALL(*device_container, load_manager());
  EXPECT_TRUE(device_container->load_manager());
  EXPECT_FALSE(device_container->lifecycle_manager_);
}

TEST_F(DeviceContainerTest, test_load_manager_lifecycle)
{
  device_container->lifecycle_operation_ = true;
  EXPECT_CALL(*device_container, load_manager());
  EXPECT_CALL(*device_container, add_node_to_executor(_));
  EXPECT_TRUE(device_container->load_manager());
  EXPECT_TRUE(device_container->lifecycle_manager_);
}

TEST_F(DeviceContainerTest, test_configure_good)
{
  device_container->set_parameter(rclcpp::Parameter("can_interface_name", "vcan0"));
  device_container->set_parameter(rclcpp::Parameter("master_config", "master.dcf"));
  device_container->set_parameter(
    rclcpp::Parameter("bus_config", "bus_configs/good_master_and_two_driver.yml"));

  EXPECT_CALL(*device_container, configure());
  EXPECT_NO_THROW(device_container->configure());
}

TEST_F(DeviceContainerTest, test_init)
{
  device_container->set_parameter(rclcpp::Parameter("can_interface_name", "vcan0"));
  device_container->set_parameter(rclcpp::Parameter("master_config", "master.dcf"));
  device_container->set_parameter(
    rclcpp::Parameter("bus_config", "bus_configs/good_master_and_two_driver.yml"));

  EXPECT_CALL(*device_container, configure()).WillOnce(Return());
  EXPECT_CALL(*device_container, load_master()).WillOnce(Return(true));
  EXPECT_CALL(*device_container, load_drivers()).WillOnce(Return(true));
  EXPECT_CALL(*device_container, load_manager()).WillOnce(Return(true));
  EXPECT_NO_THROW(device_container->init());
}

TEST_F(DeviceContainerTest, test_init_with_fparam)
{
  EXPECT_CALL(*device_container, load_master()).WillOnce(Return(true));
  EXPECT_CALL(*device_container, load_drivers()).WillOnce(Return(true));
  EXPECT_CALL(*device_container, load_manager()).WillOnce(Return(true));
  EXPECT_NO_THROW(device_container->init(
    "vcan0", "master.dcf", "bus_configs/good_master_and_two_driver.yml", ""));
}

// TEST(ComponentLoad, test_device_container_configure)
// {
//     rclcpp::init(0, nullptr);
//     auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
//     auto device_container = std::make_shared<DeviceContainer>(exec);
//     exec->add_node(device_container);

//     device_container->set_parameter(Parameter("bus_config", "bus.yml"));
//     device_container->set_parameter(Parameter("master_config", "master.dcf"));
//     device_container->set_parameter(Parameter("can_interface_name", "can0"));

//     exec->spin_some(100ms);

//     device_container->configure();

//     auto time_now = std::chrono::steady_clock::now();
//     auto time_future = time_now + 100ms;
//     while (time_future > std::chrono::steady_clock::now())
//     {
//         exec->spin_some(100ms);
//     }
//     rclcpp::shutdown();
// }

// TEST(ComponentLoad, test_load_master)
// {
//     rclcpp::init(0, nullptr);
//     auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
//     auto device_container = std::make_shared<DeviceContainer>(exec);
//     exec->add_node(device_container);

//     device_container->set_parameter(Parameter("bus_config", "bus.yml"));
//     device_container->set_parameter(Parameter("master_config", "master.dcf"));
//     device_container->set_parameter(Parameter("can_interface_name", "vcan0"));

//     exec->spin_some(100ms);

//     device_container->configure();
//     device_container->load_master();

//     std::thread spinner(
//         [exec]()
//         {
//             exec->spin();
//             RCLCPP_INFO(rclcpp::get_logger("test"), "Executor done.");
//         });
//     std::this_thread::sleep_for(500ms);
//     device_container->shutdown();
//     rclcpp::shutdown();
//     spinner.join();
//     RCLCPP_INFO(rclcpp::get_logger("test"), "rclcpp::shutdown");
// }

// TEST(ComponentLoad, test_load_component_2)
// {
//     rclcpp::init(0, nullptr);
//     auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
//     auto device_container = std::make_shared<DeviceContainer>(exec);
//     exec->add_node(device_container);

//     device_container->set_parameter(Parameter("bus_config", "bus.yml"));
//     device_container->set_parameter(Parameter("master_config", "master.dcf"));
//     device_container->set_parameter(Parameter("can_interface_name", "vcan0"));

//     std::thread spinner (
//         [exec](){
//             exec->spin();
//             RCLCPP_INFO(rclcpp::get_logger("test"), "Executor done.");
//         }
//     );
//     device_container->configure();
//     device_container->load_master();
//     EXPECT_THROW(device_container->load_drivers(), std::system_error);
//     std::this_thread::sleep_for(500ms);
//     device_container->shutdown();
//     rclcpp::shutdown();
//     spinner.join();

// }
