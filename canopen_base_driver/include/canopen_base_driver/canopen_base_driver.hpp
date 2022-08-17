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
#include <memory>

#include "canopen_base_driver/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "canopen_base_driver/lely_bridge.hpp"
#include "canopen_core/device.hpp"
#include "canopen_interfaces/msg/co_data.hpp"
#include "canopen_interfaces/srv/co_read.hpp"
#include "canopen_interfaces/srv/co_write.hpp"

namespace ros2_canopen
{
  /**
   * @brief Abstract Class for a CANopen Device Node
   *
   * This class provides the base functionality for creating a
   * CANopen device node. It provides callbacks for nmt and rpdo.
   */
  class BaseDriver : public DriverInterface
  {
  private:
    std::future<void> nmt_state_publisher_future;
    std::future<void> rpdo_publisher_future;

    void nmt_listener();
    void rdpo_listener();

  protected:
    std::shared_ptr<ros2_canopen::LelyBridge> driver;
    std::shared_ptr<ros2_canopen::ConfigurationManager> config_;

    /**
     * @brief NMT State Change Callback
     *
     * This function is called, when the NMT State of the
     * associated LelyBridge changes,
     *
     * @param [in] nmt_state New NMT state
     */
    virtual void on_nmt(canopen::NmtState nmt_state)
    {
      RCLCPP_INFO(this->get_logger(), "New NMT state %d", (int)nmt_state);
    }

    /**
     * @brief RPDO Callback
     *
     * This funciton is called when the associated
     * LelyBridge detects a change
     * on a specific object, due to an RPDO event.
     *
     * @param [in] data Changed object
     */
    virtual void on_rpdo(COData data)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Received PDO index %hu subindex %hhu data %u",
          data.index_,
          data.subindex_,
          data.data_);
    }

    explicit BaseDriver(
        const rclcpp::NodeOptions &options)
        : DriverInterface("base_driver", options) {}

  public:
    /**
     * @brief Initializer for the driver
     * 
     * Initializes the driver, adds it to the CANopen Master.
     * This function needs to be executed inside the masters
     * event loop or the masters thread!
     * 
     * @param [in] exec       The executor to be used for the driver
     * @param [in] master     The master the driver should be added to
     * @param [in] node_id    The nodeid of the device the driver commands
     */
    void init(
        ev::Executor &exec,
        canopen::AsyncMaster &master,
        uint8_t node_id,
        std::shared_ptr<ConfigurationManager> config) noexcept override;
    /**
     * @brief Removes the driver from master
     * 
     * @todo Check interface!
     * 
     * @param [in] exec     
     * @param [in] master 
     * @param [in] node_id 
     */
    void remove(
      ev::Executor &exec,
      canopen::AsyncMaster &master,
      uint8_t node_id) noexcept override;
  };
} // namespace ros2_canopen

#endif // CANOPEN_BASE_DRIVER__CANOPEN_BASE_DRIVER_HPP_