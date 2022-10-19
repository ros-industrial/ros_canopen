//    Copyright 2022 Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#ifndef CANOPEN_402_DRIVER__402_DRIVER_HPP_
#define CANOPEN_402_DRIVER__402_DRIVER_HPP_
#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
#include "canopen_core/driver_node.hpp"


namespace ros2_canopen
{
  /**
   * @brief Abstract Class for a CANopen Device Node
   *
   * This class provides the base functionality for creating a
   * CANopen device node. It provides callbacks for nmt and rpdo.
   */
  class Cia402Driver : public ros2_canopen::CanopenDriver
  {
    std::shared_ptr<node_interfaces::NodeCanopen402Driver<rclcpp::Node>> node_canopen_402_driver_;
    public:
    Cia402Driver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

    virtual bool reset_node_nmt_command()
    {
      return node_canopen_402_driver_->reset_node_nmt_command();
    }

    virtual bool start_node_nmt_command()
    {
      return node_canopen_402_driver_->start_node_nmt_command();
    }

    virtual bool tpdo_transmit(ros2_canopen::COData &data)
    {
      return node_canopen_402_driver_->tpdo_transmit(data);
    }

    virtual bool sdo_write(ros2_canopen::COData &data)
    {
      return node_canopen_402_driver_->sdo_write(data);
    }

    virtual bool sdo_read(ros2_canopen::COData &data)
    {
      return node_canopen_402_driver_->sdo_read(data);
    }
    
    void register_nmt_state_cb(std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb)
    {
      node_canopen_402_driver_->register_nmt_state_cb(nmt_state_cb);
    }

    void register_rpdo_cb(std::function<void(COData, uint8_t)> rpdo_cb)
    {
      node_canopen_402_driver_->register_rpdo_cb(rpdo_cb);
    }

    double get_speed()
    {
      return node_canopen_402_driver_->get_speed();
    }

    double get_position()
    {
      return node_canopen_402_driver_->get_position();
    }

    bool set_target(double target){
        return node_canopen_402_driver_->set_target(target);
    }

    bool init_motor(){
        return node_canopen_402_driver_->init_motor();
    }

    bool recover_motor(){
      return node_canopen_402_driver_->recover_motor();
    }

    bool halt_motor(){
      return node_canopen_402_driver_->halt_motor();
    }

    bool set_mode_position(){
        return node_canopen_402_driver_->set_mode_position();
    }

    bool set_mode_velocity(){
        return node_canopen_402_driver_->set_mode_velocity();
    }

    bool set_mode_cyclic_position(){
        return node_canopen_402_driver_->set_mode_cyclic_position();
    }

    bool set_mode_cyclic_velocity(){
        return node_canopen_402_driver_->set_mode_cyclic_velocity();
    }

    bool set_mode_torque(){
        return node_canopen_402_driver_->set_mode_torque();
    }

    uint16_t get_mode(){
        return node_canopen_402_driver_->get_mode();
    }

  };
} // namespace ros2_canopen

#endif  // CANOPEN_402_DRIVER__CANOPEN_402_DRIVER_HPP_