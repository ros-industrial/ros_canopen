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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-06-29
 *
 */
//----------------------------------------------------------------------

#ifndef CANOPEN_ROS2_CONTROL__CANOPEN_SYSTEM_HPP_
#define CANOPEN_ROS2_CONTROL__CANOPEN_SYSTEM_HPP_

#include <string>
#include <vector>

#include "canopen_core/device_container.hpp"
#include "canopen_proxy_driver/proxy_driver.hpp"
#include "canopen_ros2_control/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace canopen_ros2_control
{
// needed auxiliary struct for ros2 control double registration
struct Ros2ControlCOData
{
  ros2_canopen::COData original_data;

  double index;     // cast to uint16_t
  double subindex;  // cast to uint8_t
  double data;      // cast to uint32_t
};

struct RORos2ControlCOData : public Ros2ControlCOData
{
  void set_data(ros2_canopen::COData d)
  {
    original_data = d;

    index = static_cast<double>(original_data.index_);
    subindex = static_cast<double>(original_data.subindex_);
    data = static_cast<double>(original_data.data_);
  }
};

struct WORos2ControlCoData : public Ros2ControlCOData
{
  WORos2ControlCoData() : one_shot(std::numeric_limits<double>::quiet_NaN()) {}

  // needed internally for write-only data
  double one_shot;

  bool write_command()
  {
    bool ret_val;
    // store ret value
    ret_val = !std::isnan(one_shot);
    // reset the existing active command if one exists
    one_shot = std::numeric_limits<double>::quiet_NaN();
    return ret_val;
  }

  void prepare_data()
  {
    original_data.index_ = static_cast<uint16_t>(index);
    original_data.subindex_ = static_cast<uint8_t>(subindex);
    original_data.data_ = static_cast<uint32_t>(data);
  }
};
struct Ros2ControlNmtState
{
  Ros2ControlNmtState()
  : reset_ons(std::numeric_limits<double>::quiet_NaN()),
    reset_fbk(std::numeric_limits<double>::quiet_NaN()),
    start_ons(std::numeric_limits<double>::quiet_NaN()),
    start_fbk(std::numeric_limits<double>::quiet_NaN())
  {
  }

  void set_state(canopen::NmtState s)
  {
    original_state = s;
    state = static_cast<double>(s);
  }

  bool reset_command()
  {
    bool ret_val;
    // store ret value
    ret_val = !std::isnan(reset_ons);
    // reset the existing active command if one exists
    reset_ons = std::numeric_limits<double>::quiet_NaN();
    return ret_val;
  }

  bool start_command()
  {
    bool ret_val;
    // store ret value
    ret_val = !std::isnan(start_ons);
    // reset the existing active command if one exists
    start_ons = std::numeric_limits<double>::quiet_NaN();
    return ret_val;
  }
  canopen::NmtState original_state;

  double state;  // read-only

  // basic commands
  double reset_ons;  // write-only
  double reset_fbk;
  double start_ons;  // write-only
  double start_fbk;
};

struct CanopenNodeData
{
  Ros2ControlNmtState nmt_state;  // read-write
  RORos2ControlCOData rpdo_data;  // read-only
  WORos2ControlCoData tpdo_data;  // write-only

  WORos2ControlCoData rsdo;  // write-only
  WORos2ControlCoData wsdo;  // write-only
};

class CanopenSystem : public hardware_interface::SystemInterface
{
public:
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  CanopenSystem();
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  ~CanopenSystem();
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ros2_canopen::DeviceContainer> device_container_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  // can stuff
  std::map<uint, CanopenNodeData> canopen_data_;
  // threads
  std::unique_ptr<std::thread> spin_thread_;
  std::unique_ptr<std::thread> init_thread_;

  void spin();
  void clean();

private:
  void initDeviceContainer();
};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__CANOPEN_SYSTEM_HPP_
