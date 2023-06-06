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

#ifndef NODE_CANOPEN_MASTER_INTERFACE_HPP_
#define NODE_CANOPEN_MASTER_INTERFACE_HPP_

#include <lely/coapp/master.hpp>
#include <lely/ev/exec.hpp>
#include "canopen_core/node_interfaces/node_canopen_driver_interface.hpp"

namespace ros2_canopen
{
namespace node_interfaces
{
/**
 * @brief Node Canopen Master Interface
 *
 * This node provides the interface for NodeCanopenMaster classes
 * that provide ROS node independent CANopen functionality.
 *
 */
class NodeCanopenMasterInterface
{
public:
  /**
   * @brief Initialise Master
   *
   */
  virtual void init() = 0;

  /**
   * @brief Configure the driver
   *
   * This function should contain the configuration related things,
   * such as reading parameter data or configuration data from files.
   *
   */
  virtual void configure() = 0;

  /**
   * @brief Activate the driver
   *
   * This function should activate the driver, consequently it needs to start all timers and threads
   * necessary for the operation of the driver.
   *
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivate the driver
   *
   * This function should deactivate the driver, consequently it needs to stop all timers and
   * threads that are related to the operation of the diver.
   *
   */
  virtual void deactivate() = 0;

  /**
   * @brief Cleanup the driver
   *
   * This function needs to clean the internal state of the driver. This means
   * all data should be deleted.
   *
   */
  virtual void cleanup() = 0;

  /**
   * @brief Shutdown the driver
   *
   * This function should shutdown the driver.
   *
   */
  virtual void shutdown() = 0;
  /**
   * @brief Get the master object
   *
   * @return std::shared_ptr<lely::canopen::AsyncMaster>
   */
  virtual std::shared_ptr<lely::canopen::AsyncMaster> get_master() = 0;
  /**
   * @brief Get the executor object
   *
   * @return std::shared_ptr<lely::canopen::Executor>
   */
  virtual std::shared_ptr<lely::ev::Executor> get_executor() = 0;
};
}  // namespace node_interfaces
}  // namespace ros2_canopen
#endif
