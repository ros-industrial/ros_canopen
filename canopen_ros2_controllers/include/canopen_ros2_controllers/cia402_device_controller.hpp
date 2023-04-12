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

#include "canopen_interfaces/srv/co_target_double.hpp"
#include "canopen_ros2_controllers/canopen_proxy_controller.hpp"

static constexpr int kLoopPeriodMS = 100;
static constexpr double kCommandValue = 1.0;

namespace
{
enum Cia402CommandInterfaces
{
  INIT_CMD = CommandInterfaces::LAST_COMMAND_AUX,
  INIT_FBK,
  HALT_CMD,
  HALT_FBK,
  RECOVER_CMD,
  RECOVER_FBK,
  POSITION_MODE_CMD,
  POSITION_MODE_FBK,
  VELOCITY_MODE_CMD,
  VELOCITY_MODE_FBK,
  CYCLIC_VELOCITY_MODE_CMD,
  CYCLIC_VELOCITY_MODE_FBK,
  CYCLIC_POSITION_MODE_CMD,
  CYCLIC_POSITION_MODE_FBK,
  INTERPOLATED_POSITION_MODE_CMD,
  INTERPOLATED_POSITION_MODE_FBK,
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
  controller_interface::CallbackReturn on_init();

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period);

protected:
  inline rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr createTriggerSrv(
    const std::string & service, Cia402CommandInterfaces cmd, Cia402CommandInterfaces fbk)
  {
    // define service profile
    auto service_profile = rmw_qos_profile_services_default;
    service_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    service_profile.depth = 1;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv =
      get_node()->create_service<std_srvs::srv::Trigger>(
        service,
        [&, cmd, fbk](
          const std_srvs::srv::Trigger::Request::SharedPtr request,
          std_srvs::srv::Trigger::Response::SharedPtr response)
        {
          command_interfaces_[cmd].set_value(kCommandValue);

          while (std::isnan(command_interfaces_[fbk].get_value()) && rclcpp::ok())
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMS));
          }

          // report success
          response->success = static_cast<bool>(command_interfaces_[fbk].get_value());
          // reset to nan
          command_interfaces_[fbk].set_value(std::numeric_limits<double>::quiet_NaN());
          command_interfaces_[cmd].set_value(std::numeric_limits<double>::quiet_NaN());
        },
        service_profile);

    return srv;
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_init_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_halt_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_recover_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_position_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_torque_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_velocity_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_cyclic_velocity_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_cyclic_position_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_interpolated_position_service_;
  rclcpp::Service<canopen_interfaces::srv::COTargetDouble>::SharedPtr handle_set_target_service_;
};

}  // namespace canopen_ros2_controllers

#endif  // CANOPEN_ROS2_CONTROLLERS__CANOPEN_CIA402_CONTROLLER_HPP_
