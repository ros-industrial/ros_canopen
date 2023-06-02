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

#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
#include "controller_interface/helpers.hpp"

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
Cia402DeviceController::Cia402DeviceController()
: canopen_ros2_controllers::CanopenProxyController()
{
}

controller_interface::CallbackReturn Cia402DeviceController::on_init()
{
  if (CanopenProxyController::on_init() != controller_interface::CallbackReturn::SUCCESS)
    return controller_interface::CallbackReturn::ERROR;

  handle_init_service_ = createTriggerSrv(
    "~/init", Cia402CommandInterfaces::INIT_CMD, Cia402CommandInterfaces::INIT_FBK);

  handle_halt_service_ = createTriggerSrv(
    "~/halt", Cia402CommandInterfaces::HALT_CMD, Cia402CommandInterfaces::HALT_FBK);

  handle_recover_service_ = createTriggerSrv(
    "~/recover", Cia402CommandInterfaces::RECOVER_CMD, Cia402CommandInterfaces::RECOVER_FBK);

  handle_set_mode_position_service_ = createTriggerSrv(
    "~/position_mode", Cia402CommandInterfaces::POSITION_MODE_CMD,
    Cia402CommandInterfaces::POSITION_MODE_FBK);

  handle_set_mode_velocity_service_ = createTriggerSrv(
    "~/velocity_mode", Cia402CommandInterfaces::VELOCITY_MODE_CMD,
    Cia402CommandInterfaces::VELOCITY_MODE_FBK);

  handle_set_mode_cyclic_velocity_service_ = createTriggerSrv(
    "~/cyclic_velocity_mode", Cia402CommandInterfaces::CYCLIC_VELOCITY_MODE_CMD,
    Cia402CommandInterfaces::CYCLIC_VELOCITY_MODE_FBK);

  handle_set_mode_cyclic_position_service_ = createTriggerSrv(
    "~/cyclic_position_mode", Cia402CommandInterfaces::CYCLIC_POSITION_MODE_CMD,
    Cia402CommandInterfaces::CYCLIC_POSITION_MODE_FBK);

  handle_set_mode_interpolated_position_service_ = createTriggerSrv(
    "~/interpolated_position_mode", Cia402CommandInterfaces::INTERPOLATED_POSITION_MODE_CMD,
    Cia402CommandInterfaces::INTERPOLATED_POSITION_MODE_FBK);

  /*
  handle_set_mode_torque_service_ = createTriggerSrv("~/torque_mode",
                                                             Cia402CommandInterfaces::,
                                                             Cia402CommandInterfaces::);

  handle_set_target_service_ = createTriggerSrv("~/target", Cia402CommandInterfaces::,
                                               Cia402CommandInterfaces::);
  */

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Cia402DeviceController::command_interface_configuration() const
{
  auto command_interfaces_config = CanopenProxyController::command_interface_configuration();
  command_interfaces_config.names.push_back(joint_name_ + "/" + "init_cmd");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "init_fbk");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "halt_cmd");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "halt_fbk");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "recover_cmd");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "recover_fbk");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "position_mode_cmd");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "position_mode_fbk");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "velocity_mode_cmd");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "velocity_mode_fbk");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "cyclic_velocity_mode_cmd");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "cyclic_velocity_mode_fbk");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "cyclic_position_mode_cmd");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "cyclic_position_mode_fbk");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "interpolated_position_mode_cmd");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "interpolated_position_mode_fbk");
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration Cia402DeviceController::state_interface_configuration()
  const
{
  auto state_interfaces_config = CanopenProxyController::state_interface_configuration();
  // no new state interfaces for this controller - additional state interfaces in cia402_system
  // are position and velocity which are claimed by joint_state_broadcaster and
  // feedback based controllers
  return state_interfaces_config;
}

controller_interface::CallbackReturn Cia402DeviceController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  if (CanopenProxyController::on_configure(previous_state) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Cia402DeviceController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  if (CanopenProxyController::on_activate(previous_state) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Cia402DeviceController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  if (CanopenProxyController::on_deactivate(previous_state) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type Cia402DeviceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (CanopenProxyController::update(time, period) != controller_interface::return_type::OK)
    return controller_interface::return_type::ERROR;

  return controller_interface::return_type::OK;
}

}  // namespace canopen_ros2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  canopen_ros2_controllers::Cia402DeviceController, controller_interface::ControllerInterface)
