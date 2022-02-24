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

#ifndef BASE_DEVICE_NODE_HPP
#define BASE_DEVICE_NODE_HPP
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "proxy_device_driver/basic_device_driver.hpp"
#include "ros2_canopen_core/device.hpp"
#include "ros2_canopen_interfaces/msg/co_data.hpp"
#include "ros2_canopen_interfaces/srv/co_read.hpp"
#include "ros2_canopen_interfaces/srv/co_write.hpp"


namespace ros2_canopen
{
    /**
     * @brief Abstract Class for a DeviceNode
     * 
     * This class provides the base functionality for creating a
     * CANopen device node. It provides callbacks for nmt and rpdo.
     */
    class BaseDriver : public CANopenDriverWrapper
    {

    private:
        std::future<void> nmt_state_publisher_future;
        std::future<void> rpdo_publisher_future;

        void nmt_listener();
        void rdpo_listener();

    protected:
        std::shared_ptr<ros2_canopen::BasicDeviceDriver> driver;

        /**
         * @brief NMT State Change Callback
         * 
         * This function is called, when the NMT State of the
         * associated BasicDeviceDriver changes,
         * 
         * @param [in] nmt_state New NMT state
         */
        virtual void on_nmt(canopen::NmtState nmt_state)
        {
            RCLCPP_INFO(this->get_logger(), "on_nmt not implemented");
        }

        /**
         * @brief RPDO Callback
         * 
         * This funciton is called when the associated 
         * BasicDeviceDriver detects a change 
         * on a specific object, due to an RPDO event.
         * 
         * @param [in] data Changed object
         */
        virtual void on_rpdo(COData data)
        {
            RCLCPP_INFO(this->get_logger(), "on_rpdo not implemented");
        }

 
        explicit BaseDriver(
            const rclcpp::NodeOptions & options) : CANopenDriverWrapper("base_driver",options) {}
    
    public:
        void init(ev::Executor& exec,
            canopen::AsyncMaster& master,
            uint8_t node_id) noexcept override;
    };
}

#endif
