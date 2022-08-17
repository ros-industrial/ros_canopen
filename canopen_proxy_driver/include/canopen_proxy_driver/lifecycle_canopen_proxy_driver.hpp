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

#ifndef CANOPEN_PROXY_DRIVER__CANOPEN_PROXY_DRIVER_HPP_
#define CANOPEN_PROXY_DRIVER__CANOPEN_PROXY_DRIVER_HPP_
#include <string>

#include "canopen_proxy_driver/visibility_control.h"
#include "canopen_base_driver/lifecycle_canopen_base_driver.hpp"

namespace ros2_canopen
{
  class LifecycleProxyDriver : public LifecycleBaseDriver
  {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nmt_state_publisher;
    rclcpp::Publisher<canopen_interfaces::msg::COData>::SharedPtr rpdo_publisher;
    rclcpp::Subscription<canopen_interfaces::msg::COData>::SharedPtr tpdo_subscriber;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nmt_state_reset_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nmt_state_start_service;
    rclcpp::Service<canopen_interfaces::srv::CORead>::SharedPtr sdo_read_service;
    rclcpp::Service<canopen_interfaces::srv::COWrite>::SharedPtr sdo_write_service;

    std::mutex sdo_mtex;

  protected:
    void on_nmt(canopen::NmtState nmt_state);

    void on_tpdo(const canopen_interfaces::msg::COData::SharedPtr msg);

    void on_rpdo(COData d);

    void on_nmt_state_reset(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);

    void on_nmt_state_start(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);

    void on_sdo_read(
        const canopen_interfaces::srv::CORead::Request::SharedPtr request,
        canopen_interfaces::srv::CORead::Response::SharedPtr response);

    void on_sdo_write(
        const canopen_interfaces::srv::COWrite::Request::SharedPtr request,
        canopen_interfaces::srv::COWrite::Response::SharedPtr response);

    virtual void register_ros_interface() override;


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

  public:
    explicit LifecycleProxyDriver(const rclcpp::NodeOptions &options)
        : LifecycleBaseDriver(options) {}
        
  };
} // namespace ros2_canopen

#endif // CANOPEN_PROXY_DRIVER__CANOPEN_PROXY_DRIVER_HPP_
