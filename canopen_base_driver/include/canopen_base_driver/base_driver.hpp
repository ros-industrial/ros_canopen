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
#ifndef CANOPEN_BASE_DRIVER__CANOPEN_BASE_DRIVER_HPP_
#define CANOPEN_BASE_DRIVER__CANOPEN_BASE_DRIVER_HPP_

#include "canopen_base_driver/node_interfaces/node_canopen_base_driver.hpp"
#include "canopen_core/driver_node.hpp"

namespace ros2_canopen
{
/**
 * @brief Abstract Class for a CANopen Device Node
 *
 * This class provides the base functionality for creating a
 * CANopen device node. It provides callbacks for nmt and rpdo.
 */
class BaseDriver : public ros2_canopen::CanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopenBaseDriver<rclcpp::Node>> node_canopen_base_driver_;

public:
  BaseDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  void register_nmt_state_cb(std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb)
  {
    node_canopen_base_driver_->register_nmt_state_cb(nmt_state_cb);
  }

  void register_rpdo_cb(std::function<void(COData, uint8_t)> rpdo_cb)
  {
    node_canopen_base_driver_->register_rpdo_cb(rpdo_cb);
  }
};
}  // namespace ros2_canopen

#endif  // CANOPEN_BASE_DRIVER__CANOPEN_BASE_DRIVER_HPP_
