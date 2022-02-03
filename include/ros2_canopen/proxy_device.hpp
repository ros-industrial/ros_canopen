/*
 *  Copyright 2022 Christoph Hellmann Santos
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 * 
 */
#ifndef PROXY_DEVICE_HPP
#define PROXY_DEVICE_HPP
#include "ros2_canopen/canopen_device_base.hpp"
#include "ros2_canopen/proxy_device_node.hpp"

namespace ros2_canopen
{
    /**
     * @brief Interface of Proxy Device
     * 
     * This class is the interface used by CANopenNode to 
     * load the ProxyDevice. 
     */
    class ProxyDevice : public CANopenDevice
    {
    public:

        void registerDriver(
            std::shared_ptr<ev::Executor> exec,
            std::shared_ptr<canopen::AsyncMaster> master,
            std::shared_ptr<std::mutex> master_mutex,
            uint8_t id) override
        {
            /// Setup driver
            std::scoped_lock<std::mutex> lk(*master_mutex);
            std::string node_name = "basic_device_";
            node_name.append(std::to_string(id));
            driver_ = std::make_shared<BasicDeviceDriver>(*exec, *master, id, master_mutex);
            driver_node_ = std::make_shared<ProxyDeviceNode>(node_name, driver_);
        }

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node() override
        {
            return driver_node_->get_node_base_interface();
        }

    private:
        std::shared_ptr<ros2_canopen::BasicDeviceDriver> driver_;
        std::shared_ptr<ros2_canopen::ProxyDeviceNode> driver_node_;
    };
}
#endif