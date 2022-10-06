// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "canopen_ros2_controllers/cia402_device_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"

static constexpr int kLoopPeriodMS = 100;
static constexpr double kCommandValue = 1.0;

namespace
{  // utility


using ControllerCommandMsg = canopen_ros2_controllers::CanopenProxyController::ControllerCommandMsg;

// called from RT control loop
void reset_controller_command_msg(
  std::shared_ptr<ControllerCommandMsg> & msg, const std::string & joint_name)
{
}

}  // namespace

namespace canopen_ros2_controllers
{
Cia402DeviceController::Cia402DeviceController() {
}

controller_interface::CallbackReturn Cia402DeviceController::on_init() {
    auto ret_val =  CanopenProxyController::on_init();
    return ret_val;
}

controller_interface::InterfaceConfiguration Cia402DeviceController::command_interface_configuration() const {
    auto command_interfaces_config = CanopenProxyController::command_interface_configuration();
    command_interfaces_config.names.emplace_back("/");
    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration Cia402DeviceController::state_interface_configuration() const {
    auto state_interfaces_config = CanopenProxyController::state_interface_configuration();
    state_interfaces_config.names.emplace_back("/");
    return state_interfaces_config;
}

controller_interface::CallbackReturn
Cia402DeviceController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    auto ret_val = CanopenProxyController::on_configure(previous_state);
    return ret_val;
}

controller_interface::CallbackReturn
Cia402DeviceController::on_activate(const rclcpp_lifecycle::State &previous_state) {
    auto ret_val = CanopenProxyController::on_activate(previous_state);
    return ret_val;
}

controller_interface::CallbackReturn
Cia402DeviceController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    auto ret_val = CanopenProxyController::on_deactivate(previous_state);
    return ret_val;
}

controller_interface::return_type
Cia402DeviceController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
    auto ret_val = CanopenProxyController::update(time, period);
    return ret_val;
}

}  // namespace canopen_ros2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  canopen_ros2_controllers::Cia402DeviceController, controller_interface::ControllerInterface)
