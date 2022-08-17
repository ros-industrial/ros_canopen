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
#include <mutex>
#include <atomic>

#include "canopen_base_driver/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "canopen_base_driver/lely_bridge.hpp"
#include "canopen_core/device.hpp"
#include "canopen_interfaces/msg/co_data.hpp"
#include "canopen_interfaces/srv/co_read.hpp"
#include "canopen_interfaces/srv/co_write.hpp"
#include "canopen_interfaces/srv/co_node.hpp"

namespace ros2_canopen
{
  /**
   * @brief Abstract Class for a CANopen Device Node
   *
   * This class provides the base functionality for creating a
   * CANopen device node. It provides callbacks for nmt and rpdo.
   */
  class LifecycleBaseDriver : public LifecycleDriverInterface
  {
  protected:
    std::thread nmt_state_publisher_thread_;
    std::thread rpdo_publisher_thread_;
    

    void nmt_listener();
    void rdpo_listener();

    std::mutex driver_mutex_;
    std::shared_ptr<ros2_canopen::LelyBridge> driver_;

    virtual void start_threads() override
    {
      nmt_state_publisher_thread_ =
          std::thread(std::bind(&ros2_canopen::LifecycleBaseDriver::nmt_listener, this));

      rpdo_publisher_thread_ =
          std::thread(std::bind(&ros2_canopen::LifecycleBaseDriver::rdpo_listener, this));
    }

    virtual void join_threads() override
    {
      nmt_state_publisher_thread_.join();
      rpdo_publisher_thread_.join();
    }

    /**
     * @brief Configures the driver
     *
     * Read parameters
     * Initialise objects
     *
     * @param state
     * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state);

    /**
     * @brief Activates the driver
     *
     * Add driver to masters event loop
     *
     * @param state
     * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state);

    /**
     * @brief Deactivates the driver
     *
     * Remove driver from masters event loop
     *
     * @param state
     * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state);

    /**
     * @brief Cleanup the driver
     *
     * Delete objects
     *
     * @param state
     * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state);

    /**
     * @brief NMT State Change Callback
     *
     * Drivers that use the BaseDriver should implement this
     * function to handle NMT State changes signaled by the
     * device.
     *
     * @param [in] nmt_state New NMT state
     */
    virtual void on_nmt(canopen::NmtState nmt_state) = 0;

    /**
     * @brief RPDO Callback
     *
     * Drivers that use the BaseDriver should implement this
     * function to handle PDOs sent from the device.
     *
     * @param [in] data Changed object
     */
    virtual void on_rpdo(COData data) = 0;

    explicit LifecycleBaseDriver(
        const rclcpp::NodeOptions &options)
        : LifecycleDriverInterface("base_driver", options)
    {
    }

  public:
    virtual bool add() override;
    virtual bool remove() override;
  };
} // namespace ros2_canopen

#endif // CANOPEN_BASE_DRIVER__CANOPEN_BASE_DRIVER_HPP_
