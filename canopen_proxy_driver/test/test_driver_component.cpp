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

#include <rclcpp/executors.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <rclcpp_components/node_instance_wrapper.hpp>
#include <thread>
#include "canopen_base_driver/node_interfaces/node_canopen_base_driver.hpp"
#include "gtest/gtest.h"
using namespace rclcpp_components;

TEST(ComponentLoad, test_load_component_1)
{
  rclcpp::init(0, nullptr);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);

  std::vector<ComponentManager::ComponentResource> resources =
    manager->get_component_resources("canopen_proxy_driver");

  EXPECT_EQ(2u, resources.size());

  auto factory = manager->create_component_factory(resources[0]);
  auto instance_wrapper =
    factory->create_node_instance(rclcpp::NodeOptions().use_global_arguments(false));

  rclcpp::shutdown();
}

TEST(ComponentLoad, test_load_component_2)
{
  rclcpp::init(0, nullptr);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);

  std::vector<ComponentManager::ComponentResource> resources =
    manager->get_component_resources("canopen_proxy_driver");

  EXPECT_EQ(2u, resources.size());

  auto factory = manager->create_component_factory(resources[1]);
  auto instance_wrapper =
    factory->create_node_instance(rclcpp::NodeOptions().use_global_arguments(false));

  rclcpp::shutdown();
}
