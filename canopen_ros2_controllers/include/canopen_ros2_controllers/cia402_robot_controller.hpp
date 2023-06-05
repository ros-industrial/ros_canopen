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
#include "cia402_robot_controller_parameters.hpp"

namespace cia402_robot_controller
{
/**
 * @brief Controller for a robot with CiA402 drivers.
 *
 * This controller handles bringing up of the CiA402 drivers to operational
 * state with the right operation mode set.
 */
class Cia402RobotController : public controller_interface::ControllerInterface
{
public:
  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  Cia402RobotController();

  /**
   * @brief Initialize controller
   *
   * @details
   * This function initializes the controller. It declares the
   * parameters of the controller. This is done using the
   * generate_parameter_library.
   *
   * @return controller_interface::CallbackReturn
   */
  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Get the command interface configuration object
   *
   * @details
   * This function returns the command interface configuration.
   * This controller has uses the following command interfaces per joint:
   * - init
   * - init_feedback
   * - halt
   * - halt_feedback
   * - recover
   * - recover_feedback
   * - operation_mode
   * - operation_mode_feedback
   *
   * @return controller_interface::InterfaceConfiguration
   */
  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Get the state interface configuration object
   *
   * @details
   * This function returns the state interface configuration.
   * This controller has no state interface.
   *
   * @return CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
   */
  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Configure controller
   *
   * @details
   * This function configures the controller. It reads the
   * parameters of the controller and sets up the interfaces
   * used by the controller.
   *
   * @param previous_state
   * @return controller_interface::CallbackReturn
   */
  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate controller
   *
   * @details
   * This function activates the controller. It brings up the
   * CiA402 drivers to operational state with the right operation
   * mode set.
   *
   * @param previous_state
   * @return controller_interface::CallbackReturn
   */
  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate controller
   *
   *
   * @param previous_state
   * @return controller_interface::CallbackReturn
   */
  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Update controller
   *
   * @details
   * This function updates the controller. It does nothing.
   *
   * @param time
   * @param period
   * @return controller_interface::return_type
   */
  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_recover_service_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  controller_interface::InterfaceConfiguration command_interfaces_config_;
  controller_interface::InterfaceConfiguration state_interfaces_config_;

  void declare_parameters();
  controller_interface::CallbackReturn read_parameters();
  void activate();

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cia402_robot_controller

#endif  // CANOPEN_ROS2_CONTROLLERS__CANOPEN_CIA402_CONTROLLER_HPP_
