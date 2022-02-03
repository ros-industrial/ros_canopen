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
#include "ros2_canopen/proxy_device_node.hpp"

using namespace ros2_canopen;

void ProxyDeviceNode::on_nmt_state_reset(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    driver->nmt_command(canopen::NmtCommand::RESET_NODE);
    response->success = true;
}

void ProxyDeviceNode::on_sdo_read(
    const ros2_canopen_interfaces::srv::CORead::Request::SharedPtr request,
    ros2_canopen_interfaces::srv::CORead::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Slave %hhu: SDO Read Call index=0x%x subindex=%hhu bits=%hhu",
                this->driver->get_id(), request->index, request->subindex, request->type);

    // Only allow one SDO request concurrently
    std::scoped_lock<std::mutex> lk(sdo_mtex);
    // Prepare Data read
    COData data = {request->index, request->subindex, 0U, static_cast<CODataTypes>(request->type)};
    // Send read request
    auto f = driver->async_sdo_read(data);
    // Wait for response
    f.wait();
    // Process response
    response->data = f.get().data_;
    response->success = true;
}

void ProxyDeviceNode::on_sdo_write(
    const ros2_canopen_interfaces::srv::COWrite::Request::SharedPtr request,
    ros2_canopen_interfaces::srv::COWrite::Response::SharedPtr response)
{
    RCLCPP_INFO(this->get_logger(), "Slave %hhu: SDO Read Call index=0x%x subindex=%hhu bits=%hhu data=%u",
                this->driver->get_id(), request->index, request->subindex, request->type, request->data);

    // Only allow one SDO request concurrently
    std::scoped_lock<std::mutex> lk(sdo_mtex);

    // Prepare Data
    COData d = {request->index, request->subindex, request->data, static_cast<CODataTypes>(request->type)};
    // Send write request
    auto f = driver->async_sdo_write(d);
    //Wait for request to complete
    f.wait();

    // Process response
    response->success = f.get();
}

void ProxyDeviceNode::on_rpdo(COData d)
{
    RCLCPP_INFO(this->get_logger(),
                "Slave %hhu: Sent PDO index %hu, subindex %hhu, data %x",
                this->driver->get_id(),
                d.index_,
                d.subindex_,
                d.data_);
    auto message = ros2_canopen_interfaces::msg::COData();
    message.index = d.index_;
    message.subindex = d.subindex_;
    message.data = d.data_;
    message.type = static_cast<uint8_t>(d.type_);
    rpdo_publisher->publish(message);
}

void ProxyDeviceNode::on_tpdo(const ros2_canopen_interfaces::msg::COData::SharedPtr msg)
{
    COData data = {msg->index, msg->subindex, msg->data, static_cast<CODataTypes>(msg->type)};
    driver->tpdo_transmit(data);
}

void ProxyDeviceNode::on_nmt(canopen::NmtState nmt_state)
{
    auto message = std_msgs::msg::String();

    switch (nmt_state)
    {
    case canopen::NmtState::BOOTUP:
        message.data = "BOOTUP";
        break;
    case canopen::NmtState::PREOP:
        message.data = "PREOP";
        break;
    case canopen::NmtState::RESET_COMM:
        message.data = "RESET_COMM";
        break;
    case canopen::NmtState::RESET_NODE:
        message.data = "RESET_NODE";
        break;
    case canopen::NmtState::START:
        message.data = "START";
        break;
    case canopen::NmtState::STOP:
        message.data = "STOP";
        break;
    case canopen::NmtState::TOGGLE:
        message.data = "TOGGLE";
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown NMT State.");
        message.data = "ERROR";
        break;
    }
    RCLCPP_INFO(this->get_logger(),
                "Slave %hhu: Switched NMT state to %s",
                this->driver->get_id(),
                message.data.c_str());

    nmt_state_publisher->publish(message);
}

CallbackReturn ProxyDeviceNode::on_configure_app(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}
CallbackReturn ProxyDeviceNode::on_activate_app(const rclcpp_lifecycle::State &state)
{
    nmt_state_publisher->on_activate();
    rpdo_publisher->on_activate();
    return CallbackReturn::SUCCESS;
}
CallbackReturn ProxyDeviceNode::on_deactivate_app(const rclcpp_lifecycle::State &state)
{
    nmt_state_publisher->on_deactivate();
    rpdo_publisher->on_deactivate();
    return CallbackReturn::SUCCESS;
}
CallbackReturn ProxyDeviceNode::on_cleanup_app(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}
CallbackReturn ProxyDeviceNode::on_shuttdown_app(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}
