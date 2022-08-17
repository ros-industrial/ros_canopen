//    Copyright 2022 Harshavadan Deshpande
//                   Christoph Hellmann Santos
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
#ifndef LIFECYCLE_MASTER_NODE_HPP
#define LIFECYCLE_MASTER_NODE_HPP

#include <memory>
#include <thread>
#include <atomic>

#include <lifecycle_msgs/msg/state.hpp>

#include "canopen_core/exchange.hpp"
#include "canopen_core/device.hpp"
#include "canopen_core/lely_master_bridge.hpp"
#include "canopen_interfaces/srv/co_write_id.hpp"
#include "canopen_interfaces/srv/co_read_id.hpp"

namespace ros2_canopen
{
    class LifecycleMasterNode : public LifecycleMasterInterface
    {
    protected:
        std::atomic<bool> activated;
        std::shared_ptr<LelyMasterBridge> master_;
        std::unique_ptr<lely::io::IoGuard> io_guard_;
        std::unique_ptr<lely::io::Context> ctx_;
        std::unique_ptr<lely::io::Poll> poll_;
        std::unique_ptr<lely::ev::Loop> loop_;
        std::shared_ptr<lely::ev::Executor> exec_;
        std::unique_ptr<lely::io::Timer> timer_;
        std::unique_ptr<lely::io::CanController> ctrl_;
        std::unique_ptr<lely::io::CanChannel> chan_;
        std::unique_ptr<lely::io::SignalSet> sigset_;
        std::thread spinner_;

        rclcpp::Service<canopen_interfaces::srv::COReadID>::SharedPtr sdo_read_service;
        rclcpp::Service<canopen_interfaces::srv::COWriteID>::SharedPtr sdo_write_service;

    public:
        LifecycleMasterNode(
            const rclcpp::NodeOptions &node_options
            ) : LifecycleMasterInterface("master", node_options)
        {
        }

        /**
         * @brief Initialises the LifecycleMasterNode
         * 
         * As LifecycleMasterNode is a component this function enables passing data  to the
         * node that would usually be passed via the constructor.
         * 
         */
        void init() override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & state);
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State & state);

        /**
         * @brief Add a device driver
         * 
         * This function only has an effect if the LifecycleMasterNode is in active state.
         * The function will add a driver for a specific node id to the CANopen
         * event loop.
         * 
         * @param node_instance 
         * @param node_id 
         */
        void init_driver(std::shared_ptr<ros2_canopen::LifecycleDriverInterface> node_instance, uint8_t node_id) override;

        /**
         * @brief Read Service Data Object
         * 
         * This Service is only available when the node is in active lifecycle state.
         * It will return with success false in any other lifecycle state and log an
         * RCLCPP_ERROR.
         * 
         * @param request 
         * @param response 
         */
        void on_sdo_read(
            const std::shared_ptr<canopen_interfaces::srv::COReadID::Request> request,
            std::shared_ptr<canopen_interfaces::srv::COReadID::Response> response);

        /**
         * @brief Write Service Data Object
         * 
         * This service is only available when the node is in active lifecycle state.
         * It will return with success false in any other lifecycle state and log an
         * RCLCPP_ERROR.
         * 
         * @param request 
         * @param response 
         */
        void on_sdo_write(
            const std::shared_ptr<canopen_interfaces::srv::COWriteID::Request> request,
            std::shared_ptr<canopen_interfaces::srv::COWriteID::Response> response);
    };
}


#endif