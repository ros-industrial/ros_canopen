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
#ifndef BASIC_DEVICE_HPP
#define BASIC_DEVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ros2_canopen_core/basic_device_driver.hpp"
#include "ros2_canopen_interfaces/msg/co_data.hpp"
#include "ros2_canopen_interfaces/srv/co_read.hpp"
#include "ros2_canopen_interfaces/srv/co_write.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros2_canopen
{
    /**
     * @brief Abstract Class for a DeviceNode
     * 
     * This class provides the base functionality for creating a
     * CANopen device node. It provides callbacks for nmt and rpdo.
     */
    class BasicDeviceNode : public rclcpp_lifecycle::LifecycleNode
    {

    private:
        std::future<void> nmt_state_publisher_future;
        std::future<void> rpdo_publisher_future;

        void nmt_listener();
        void rdpo_listener();

        CallbackReturn
        on_configure(const rclcpp_lifecycle::State &state) override;

        CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state) override;

        CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &state) override;

        CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &state) override;

        CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &state) override;

    protected:
        std::shared_ptr<ros2_canopen::BasicDeviceDriver> driver;
        std::atomic<bool> configured;

        /**
         * @brief NMT State Change Callback
         * 
         * This function is called, when the NMT State of the
         * associated BasicDeviceDriver changes,
         * 
         * @param [in] nmt_state New NMT state
         */
        virtual void on_nmt(canopen::NmtState nmt_state) = 0;

        /**
         * @brief RPDO Callback
         * 
         * This funciton is called when the associated 
         * BasicDeviceDriver detects a change 
         * on a specific object, due to an RPDO event.
         * 
         * @param [in] data Changed object
         */
        virtual void on_rpdo(COData data) = 0;

        /**
         * @brief Configure Callback
         * 
         * This is called after BasicDeviceNode was configured.
         * 
         * @param state 
         * @return CallbackReturn 
         */
        virtual CallbackReturn on_configure_app(const rclcpp_lifecycle::State &state) = 0;

        /**
         * @brief Activate Callback
         * 
         * This is called after BasicDeviceNode was activated.
         * 
         * @param state 
         * @return CallbackReturn 
         */
        virtual CallbackReturn on_activate_app(const rclcpp_lifecycle::State &state) = 0;
        /**
         * @brief Deactivate Callback
         * 
         * This is called after BasicDeviceNode was deactivated.
         * 
         * @param state 
         * @return CallbackReturn 
         */
        virtual CallbackReturn on_deactivate_app(const rclcpp_lifecycle::State &state) = 0;

        /**
         * @brief Cleanup Callback
         * 
         * This is called after BasicDeviceNode was cleaned up.
         * 
         * @param state 
         * @return CallbackReturn 
         */
        virtual CallbackReturn on_cleanup_app(const rclcpp_lifecycle::State &state) = 0;

        /**
         * @brief Shutdown Callback
         * 
         * This is called after BasicDeviceNode was shutdown.
         * 
         * @param state 
         * @return CallbackReturn 
         */
        virtual CallbackReturn on_shuttdown_app(const rclcpp_lifecycle::State &state) = 0;

        explicit BasicDeviceNode(
            const std::string &node_name,
            std::shared_ptr<BasicDeviceDriver> driver,
            bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(
                  node_name,
                  rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms).use_global_arguments(false))
        {
            this->driver = driver;
        };
    };





}

#endif