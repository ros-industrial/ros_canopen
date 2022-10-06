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

#ifndef CANOPEN_ROS2_CONTROLLERS__CANOPEN_CIA402_CONTROLLER_HPP_
#define CANOPEN_ROS2_CONTROLLERS__CANOPEN_CIA402_CONTROLLER_HPP_

#include "canopen_ros2_controllers/canopen_proxy_controller.hpp"

namespace
{
enum Cia402CommandInterfaces
{
  FIRST_COMMAND = CommandInterfaces::LAST_COMMAND_AUX,
};

enum Cia402StateInterfaces
{
  FIRST_STATE = StateInterfaces::LAST_STATE_AUX,
};

}  // namespace

namespace canopen_ros2_controllers
{

class Cia402DeviceController : public canopen_ros2_controllers::CanopenProxyController
{
public:
  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  Cia402DeviceController();

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() ;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const ;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const ;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) ;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) ;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) ;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) ;


protected:

};

}  // namespace canopen_ros2_controllers

#endif  // CANOPEN_ROS2_CONTROLLERS__CANOPEN_CIA402_CONTROLLER_HPP_
