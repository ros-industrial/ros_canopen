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
#include "ros2_canopen_core/basic_device_node.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace lely;
using namespace ros2_canopen;

void BasicDeviceNode::nmt_listener()
{
    while (configured.load())
    {
        auto f = driver->async_request_nmt();
        f.wait();
        on_nmt(f.get());
    }
}

void BasicDeviceNode::rdpo_listener()
{
    while (configured.load())
    {
        auto f = driver->async_request_rpdo();
        f.wait();
        on_rpdo(f.get());
    }
}

CallbackReturn
BasicDeviceNode::on_configure(const rclcpp_lifecycle::State &state)
{
    configured.store(true);
    nmt_state_publisher_future = std::async(std::launch::async, std::bind(&ros2_canopen::BasicDeviceNode::nmt_listener, this));
    rpdo_publisher_future = std::async(std::launch::async, std::bind(&ros2_canopen::BasicDeviceNode::rdpo_listener, this));
    return on_configure_app(state);
}

CallbackReturn
BasicDeviceNode::on_activate(const rclcpp_lifecycle::State &state)
{
    auto res = on_activate_app(state);
    activated.store(true);
    return res;
}

CallbackReturn
BasicDeviceNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    activated.store(false);
    return on_deactivate_app(state);
}

CallbackReturn
BasicDeviceNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
    configured.store(false);
    nmt_state_publisher_future.wait();
    rpdo_publisher_future.wait();
    return on_cleanup_app(state);
}

CallbackReturn
BasicDeviceNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    return on_shuttdown_app(state);
}
