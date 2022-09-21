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
#ifndef MASTER_NODE_HPP
#define MASTER_NODE_HPP

#include <memory>
#include <thread>


#include "canopen_core/exchange.hpp"
#include "canopen_core/device.hpp"
#include "canopen_core/lely_master_bridge.hpp"
#include "canopen_interfaces/srv/co_write_id.hpp"
#include "canopen_interfaces/srv/co_read_id.hpp"
namespace ros2_canopen
{
    class MasterNode : public MasterInterface
    {
    protected:
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
        MasterNode(
            const rclcpp::NodeOptions &node_options
            ) : MasterInterface("master", node_options)
        {
        }

        /**
         * @brief Initialize master
         * 
         * @param [in] dcf_txt              Path to the DCF file
         * @param [in] dcf_bin              Path to the DCF Bin file
         * @param [in] can_interface_name   Name of the can interface
         * @param [in] nodeid               CANopen node id
         * @param [in] config               Pointer to the Configuration Manager
         */
        void init(std::string dcf_txt, std::string dcf_bin, std::string can_interface_name, uint8_t nodeid, 
                    std::shared_ptr<ConfigurationManager> config) override;

        /**
         * @brief Add a driver to the master
         * 
         * @param [in] node_instance        Instance of the driver to add
         * @param [in] node_id              CANopen node id of the target device
         */
        void add_driver(std::shared_ptr<ros2_canopen::DriverInterface> node_instance, uint8_t node_id) override;

        /**
         * @brief Remove a driver from the master
         * 
         * @param [in] node_instance        Instance of the driver to add
         * @param [in] node_id              CANopen node id of the target device
         */
        void remove_driver(std::shared_ptr<ros2_canopen::DriverInterface> node_instance, uint8_t node_id) override;

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