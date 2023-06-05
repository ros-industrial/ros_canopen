// Copyright (c) 2023, Fraunhofer IPA
// Copyright (c) 2022, StoglRobotics
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef CANOPEN_ROS2_CONTROL__CIA402_DATA_HPP_
#define CANOPEN_ROS2_CONTROL__CIA402_DATA_HPP_
#include <yaml-cpp/yaml.h>
#include <cstdint>
#include <memory>
#include <string>
#include "canopen_402_driver/base.hpp"
#include "canopen_402_driver/cia402_driver.hpp"
#include "canopen_ros2_control/helpers.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace ros2_canopen;
namespace canopen_ros2_control
{
struct Cia402Data
{
  uint8_t node_id;
  std::string joint_name;
  std::shared_ptr<ros2_canopen::Cia402Driver> driver;
  std::map<std::string, ros2_canopen::MotorBase::OperationMode> command_interface_to_operation_mode;
  std::vector<std::string> interfaces;
  std::vector<std::string> interfaces_to_start;
  std::vector<std::string> interfaces_to_running;
  std::vector<std::string> interfaces_to_stop;
  YAML::Node config;

  // FROM MOTOR
  double actual_position = std::numeric_limits<double>::quiet_NaN();
  double actual_velocity = std::numeric_limits<double>::quiet_NaN();

  // TO MOTOR
  double target_position = std::numeric_limits<double>::quiet_NaN();
  double target_velocity = std::numeric_limits<double>::quiet_NaN();
  double target_torque = std::numeric_limits<double>::quiet_NaN();

  bool init_data(hardware_interface::ComponentInfo & joint, std::string device_dump)
  {
    joint_name = joint.name;
    auto config = YAML::Load(device_dump);

    if (!config["node_id"])
    {
      RCLCPP_ERROR(rclcpp::get_logger(joint_name), "No node id for '%s'", joint.name.c_str());
      return false;
    }

    node_id = config["node_id"].as<uint16_t>();
    RCLCPP_ERROR(
      rclcpp::get_logger(joint_name), "Node id for '%s' is '%u'", joint.name.c_str(), node_id);

    if (config["position_mode"])
    {
      auto position_mode =
        (ros2_canopen::MotorBase::OperationMode)config["position_mode"].as<uint>();
      RCLCPP_INFO(
        rclcpp::get_logger(joint_name), "Registered position_mode '%u' for '%s'", position_mode,
        joint.name.c_str());
      command_interface_to_operation_mode.emplace(
        std::pair(joint.name + "/" + "position", position_mode));
    }
    if (config["velocity_mode"])
    {
      auto velocity_mode =
        (ros2_canopen::MotorBase::OperationMode)config["velocity_mode"].as<uint>();
      RCLCPP_INFO(
        rclcpp::get_logger(joint_name), "Registered velocity_mode '%u' for '%s'", velocity_mode,
        joint.name.c_str());
      command_interface_to_operation_mode.emplace(
        std::pair(joint.name + "/" + "velocity", velocity_mode));
    }
    if (config["effort_mode"])
    {
      auto effort_mode = (ros2_canopen::MotorBase::OperationMode)config["effort_mode"].as<uint>();
      RCLCPP_INFO(
        rclcpp::get_logger(joint_name), "Registered effort_mode '%u' for '%s'", effort_mode,
        joint.name.c_str());
      command_interface_to_operation_mode.emplace(
        std::pair(joint.name + "/" + "effort", effort_mode));
    }
    return true;
  }

  void export_state_interface(std::vector<hardware_interface::StateInterface> & state_interfaces)
  {
    // actual position
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_name, hardware_interface::HW_IF_POSITION, &actual_position));

    // actual speed
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_name, hardware_interface::HW_IF_VELOCITY, &actual_velocity));
  }

  void export_command_interface(
    std::vector<hardware_interface::CommandInterface> & command_interfaces)
  {
    if (
      command_interface_to_operation_mode.find(joint_name + "/" + "position") !=
      command_interface_to_operation_mode.end())
    {
      // target position
      command_interfaces.emplace_back(
        joint_name, hardware_interface::HW_IF_POSITION, &target_position);
      interfaces.push_back(joint_name + "/" + "position");
    }
    if (
      command_interface_to_operation_mode.find(joint_name + "/" + "velocity") !=
      command_interface_to_operation_mode.end())
    {
      // target velocity
      command_interfaces.emplace_back(
        joint_name, hardware_interface::HW_IF_VELOCITY, &target_velocity);
      interfaces.push_back(joint_name + "/" + "velocity");
    }
    if (
      command_interface_to_operation_mode.find(joint_name + "/" + "effort") !=
      command_interface_to_operation_mode.end())
    {
      // target effort
      command_interfaces.emplace_back(
        joint_name, hardware_interface::HW_IF_EFFORT, &target_position);
      interfaces.push_back(joint_name + "/" + "effort");
    }
  }

  void read_state()
  {
    actual_position = driver->get_position();
    actual_velocity = driver->get_speed();
  }

  void write_target()
  {
    const uint16_t & mode = driver->get_mode();
    switch (mode)
    {
      case MotorBase::No_Mode:
        break;
      case MotorBase::Profiled_Position:
      case MotorBase::Cyclic_Synchronous_Position:
      case MotorBase::Interpolated_Position:
        driver->set_target(target_position);
        break;
      case MotorBase::Profiled_Velocity:
      case MotorBase::Cyclic_Synchronous_Velocity:
        driver->set_target(target_velocity);
        break;
      case MotorBase::Profiled_Torque:
        driver->set_target(target_torque);
        break;
      default:
        RCLCPP_INFO(rclcpp::get_logger("robot_system_interface"), "Mode not supported");
    }
  }

  bool perform_switch()
  {
    if (interfaces_to_start.size() > 1)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(joint_name),
        "Trying to start multiple command interfaces at once for joint '%s'", joint_name.c_str());
      return false;
    }
    if (interfaces_to_start.size() == 0)
    {
      return true;
    }
    auto mode = command_interface_to_operation_mode[interfaces_to_start[0]];
    interfaces_to_running.clear();
    interfaces_to_running.push_back(interfaces_to_start[0]);
    interfaces_to_stop.clear();
    interfaces_to_start.clear();
    RCLCPP_INFO(
      rclcpp::get_logger(joint_name),
      "Switching to '%s' command mode with CIA402 operation mode '%u'",
      interfaces_to_running[0].c_str(), mode);
    return driver->set_operation_mode(mode);
  }

  bool check_id(uint8_t id)
  {
    if (node_id == id)
    {
      return true;
    }
    return false;
  }
};
}  // namespace canopen_ros2_control
#endif
