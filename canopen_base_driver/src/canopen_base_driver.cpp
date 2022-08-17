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
#include <memory>
#include "canopen_base_driver/canopen_base_driver.hpp"

using namespace lely;
using namespace ros2_canopen;

void BaseDriver::nmt_listener()
{
  while (rclcpp::ok()) {
    auto f = driver->async_request_nmt();
    f.wait();
    on_nmt(f.get());
  }
}

void BaseDriver::rdpo_listener()
{
  while (rclcpp::ok()) {
    auto f = driver->async_request_rpdo();
    f.wait();
    on_rpdo(f.get());
  }
}

void BaseDriver::init(
  ev::Executor & exec,
  canopen::AsyncMaster & master,
  uint8_t node_id,
  std::shared_ptr<ConfigurationManager> config) noexcept
{
  config_ = config;
  driver =
    std::make_shared<ros2_canopen::LelyBridge>(exec, master, node_id);
  nmt_state_publisher_future =
    std::async(
    std::launch::async,
    std::bind(&ros2_canopen::BaseDriver::nmt_listener, this)
    );
  rpdo_publisher_future =
    std::async(
    std::launch::async,
    std::bind(&ros2_canopen::BaseDriver::rdpo_listener, this)
    );
  driver->Boot();
}

void BaseDriver::remove(
  ev::Executor & exec,
  canopen::AsyncMaster & master,
  uint8_t node_id) noexcept
{
  driver.reset();
}