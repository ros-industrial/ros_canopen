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

/*
using ControllerCommandMsg = canopen_ros2_controllers::CanopenProxyController::ControllerCommandMsg;

// called from RT control loop
void reset_controller_command_msg(
  std::shared_ptr<ControllerCommandMsg> & msg, const std::string & joint_name)
{
  msg->index = 0u;
  msg->subindex = 0u;
  msg->type = 0u;
  msg->data = 0u;
}
bool propagate_controller_command_msg(
  std::shared_ptr<ControllerCommandMsg> & msg)
{
  // TODO (livanov93): add better logic to decide if a message
  //  should be propagated to the bus
  // check if it is 8 = uint8_t or 16 = uint16_t or 32 = uint32_t
  return msg->type == 8 || msg->type == 16 || msg->type == 32;
}
*/
}  // namespace

namespace canopen_ros2_controllers
{
Cia402DeviceController::Cia402DeviceController() {
}

controller_interface::CallbackReturn Cia402DeviceController::on_init() {
    return CanopenProxyController::on_init();
}

controller_interface::InterfaceConfiguration Cia402DeviceController::command_interface_configuration() const {
    return CanopenProxyController::command_interface_configuration();
}

controller_interface::InterfaceConfiguration Cia402DeviceController::state_interface_configuration() const {
    return CanopenProxyController::state_interface_configuration();
}

controller_interface::CallbackReturn
Cia402DeviceController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    return CanopenProxyController::on_configure(previous_state);
}

controller_interface::CallbackReturn
Cia402DeviceController::on_activate(const rclcpp_lifecycle::State &previous_state) {
    return CanopenProxyController::on_activate(previous_state);
}

controller_interface::CallbackReturn
Cia402DeviceController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    return CanopenProxyController::on_deactivate(previous_state);
}

controller_interface::return_type
Cia402DeviceController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
    return CanopenProxyController::update(time, period);
}

}  // namespace canopen_ros2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  canopen_ros2_controllers::Cia402DeviceController, controller_interface::ControllerInterface)
