#include "canopen_master_driver/node_interfaces/node_canopen_basic_master.hpp"
#include "gtest/gtest.h"
#include <thread>
#include <rclcpp/executors.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <rclcpp_components/node_instance_wrapper.hpp>
#include <rclcpp_components/node_factory.hpp>
using namespace rclcpp_components;

TEST(MasterDriverComponent, test_load_lifecycle_master_driver)
{
  rclcpp::init(0, nullptr);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);

  std::vector<ComponentManager::ComponentResource> resources = 
    manager->get_component_resources("canopen_master_driver");

  EXPECT_EQ(2u, resources.size());
  
  auto factory = manager->create_component_factory(resources[0]);
  auto instance_wrapper = factory->create_node_instance(rclcpp::NodeOptions().use_global_arguments(false));

  rclcpp::shutdown();
}

TEST(MasterDriverComponent, test_load_master_driver)
{
  rclcpp::init(0, nullptr);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);

  std::vector<ComponentManager::ComponentResource> resources = 
    manager->get_component_resources("canopen_master_driver");

  EXPECT_EQ(2u, resources.size());
  
  auto factory = manager->create_component_factory(resources[1]);
  auto instance_wrapper = factory->create_node_instance(rclcpp::NodeOptions().use_global_arguments(false));

  rclcpp::shutdown();
}
