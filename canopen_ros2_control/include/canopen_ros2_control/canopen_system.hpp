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

#include "canopen_ros2_control/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <rclcpp/executors.hpp> // for MultiThreadedExecutor
#include <canopen_core/device_manager.hpp> // for DeviceManager



namespace canopen_ros2_control
{
    // needed auxiliary struct for ros2 control double registration
    struct Ros2ControlCOData{
        ros2_canopen::COData original_data;

        double index; // cast to uint16_t
        double subindex; // cast to uint8_t
        double data;    // cast to uint32_t
        double type;// cast to uint8_t

    };

    struct RORos2ControlCOData: public Ros2ControlCOData{

        void set_data(ros2_canopen::COData d){
            original_data = d;

            index = static_cast<double>(original_data.index_);
            subindex = static_cast<double>(original_data.subindex_);
            data = static_cast<double>(original_data.data_);
            type = static_cast<double>(original_data.type_);

        }
    };

    struct WORos2ControlCoData: public Ros2ControlCOData{

        WORos2ControlCoData(): one_shot(std::numeric_limits<double>::infinity()){}

        // needed internally for write-only data
        double one_shot;

        bool write_command(){
            bool ret_val;
            // store ret value
            ret_val = (one_shot != std::numeric_limits<double>::infinity());
            // reset the existing active command if one exists
            one_shot = std::numeric_limits<double>::infinity();
            return ret_val;
        }

        void prepare_data(){

            original_data.index_ = static_cast<uint16_t>(index);
            original_data.subindex_ = static_cast<uint8_t>(subindex);
            original_data.data_ =   static_cast<uint32_t>(data);
            original_data.type_ = static_cast<ros2_canopen::CODataTypes>(type);
        }
    };
    struct Ros2ControlNmtState{

        Ros2ControlNmtState(): reset_ons(std::numeric_limits<double>::infinity()), start_ons(std::numeric_limits<double>::infinity()){}


        void set_state(canopen::NmtState s){
            original_state = s;
            state = static_cast<double>(s);
        }

        bool reset_command(){
            bool ret_val;
            // store ret value
            ret_val = (reset_ons != std::numeric_limits<double>::infinity());
            // reset the existing active command if one exists
            reset_ons = std::numeric_limits<double>::infinity();
            return ret_val;
        }

        bool start_command(){
            bool ret_val;
            // store ret value
            ret_val = (start_ons != std::numeric_limits<double>::infinity());
            // reset the existing active command if one exists
            start_ons = std::numeric_limits<double>::infinity();
            return ret_val;
        }
        canopen::NmtState original_state;

        double state; //read-only

        // basic commands
        double reset_ons; // write-only
        double start_ons; // write-only

    };

    struct CanopenNodeData{
        Ros2ControlNmtState nmt_state; // read-write
        RORos2ControlCOData rpdo_data; // read-only
        WORos2ControlCoData tpdo_data; // write-only

        WORos2ControlCoData rsdo; // write-only
        WORos2ControlCoData wsdo; // write-only

    };

using namespace ros2_canopen;
class CanopenSystem : public hardware_interface::SystemInterface
{
public:
    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    ~CanopenSystem();
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  std::shared_ptr<DeviceManager> device_manager_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<rclcpp_components::ComponentManager> component_manager_;
  std::shared_ptr<rclcpp::Node> node_;

  // can stuff
  std::map<uint, CanopenNodeData> canopen_data_;

  std::unique_ptr<std::thread> spin_thread_;
  std::unique_ptr<std::thread> init_thread_;
  void spin();
  void initDeviceManager();
};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__CANOPEN_SYSTEM_HPP_
