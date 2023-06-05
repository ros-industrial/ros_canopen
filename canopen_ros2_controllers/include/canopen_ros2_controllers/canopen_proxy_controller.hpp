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

#ifndef CANOPEN_ROS2_CONTROLLERS__CANOPEN_PROXY_CONTROLLER_HPP_
#define CANOPEN_ROS2_CONTROLLERS__CANOPEN_PROXY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "canopen_interfaces/msg/co_data.hpp"
#include "canopen_interfaces/srv/co_read.hpp"
#include "canopen_interfaces/srv/co_write.hpp"
#include "canopen_ros2_controllers/visibility_control.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
enum CommandInterfaces
{
  TPDO_INDEX,
  TPDO_SUBINDEX,
  TPDO_DATA,
  TPDO_ONS,
  NMT_RESET,
  NMT_RESET_FBK,
  NMT_START,
  NMT_START_FBK,
  LAST_COMMAND_AUX,
};

enum StateInterfaces
{
  RPDO_INDEX,
  RPDO_SUBINDEX,
  RPDO_DATA,
  NMT_STATE,
  LAST_STATE_AUX,
};

}  // namespace

namespace canopen_ros2_controllers
{

class CanopenProxyController : public controller_interface::ControllerInterface
{
public:
  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  CanopenProxyController();

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerCommandMsg = canopen_interfaces::msg::COData;
  using ControllerStartResetSrvType = std_srvs::srv::Trigger;
  using ControllerSDOReadSrvType = canopen_interfaces::srv::CORead;
  using ControllerSDOWriteSrvType = canopen_interfaces::srv::COWrite;
  using ControllerNMTStateMsg = std_msgs::msg::String;

protected:
  std::string joint_name_;

  // Command subscribers
  // TPDO subscription
  rclcpp::Subscription<ControllerCommandMsg>::SharedPtr tpdo_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandMsg>> input_cmd_;

  // NMT state publisher
  using ControllerNmtStateRTPublisher = realtime_tools::RealtimePublisher<ControllerNMTStateMsg>;
  rclcpp::Publisher<ControllerNMTStateMsg>::SharedPtr nmt_state_pub_;
  std::unique_ptr<ControllerNmtStateRTPublisher> nmt_state_rt_publisher_;
  std::string nmt_state_actual_ = "BOOTUP";

  // RPDO publisher
  using ControllerRPDOPRTublisher = realtime_tools::RealtimePublisher<ControllerCommandMsg>;
  rclcpp::Publisher<ControllerCommandMsg>::SharedPtr rpdo_pub_;
  std::unique_ptr<ControllerRPDOPRTublisher> rpdo_rt_publisher_;

  // NMT reset service
  rclcpp::Service<ControllerStartResetSrvType>::SharedPtr nmt_state_reset_service_;
  // NMT start service
  rclcpp::Service<ControllerStartResetSrvType>::SharedPtr nmt_state_start_service_;
  // SDO read service
  rclcpp::Service<ControllerSDOReadSrvType>::SharedPtr sdo_read_service_;
  // SDO write service
  rclcpp::Service<ControllerSDOWriteSrvType>::SharedPtr sdo_write_service_;
};

}  // namespace canopen_ros2_controllers

#endif  // CANOPEN_ROS2_CONTROLLERS__CANOPEN_PROXY_CONTROLLER_HPP_
