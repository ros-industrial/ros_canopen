/****************************************************************
 *
 * Copyright (c) 2014
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: canopen_402
 * ROS stack name: canopen_402
 * ROS package name: canopen_402
 * Description: This class implements the CANopen device profile for
 * drives and motion control
 * CiA (r) 402
 * Standardized in IEC 61800-7-201 and IEC 61800-7-301
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Thiago de Freitas, email:tdf@ipa.fhg.de
 * Supervised by: Thiago de Freitas, email:tdf@ipa.fhg.de
 *
 * Date of creation: July 2014
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef CANOPEN_402_CANOPEN_402_H
#define CANOPEN_402_CANOPEN_402_H

#include <canopen_master/canopen.h>
#include <string>
#include <vector>

namespace canopen
{
class Node_402 : public canopen::Layer
{
public:
  Node_402(boost::shared_ptr <canopen::Node> n, const std::string &name) : Layer(name), n_(n)
  {
    configureEntries();
    status_word_mask.set(SW_Ready_To_Switch_On);
    status_word_mask.set(SW_Switched_On);
    status_word_mask.set(SW_Operation_enabled);
    status_word_mask.set(SW_Fault);
    status_word_mask.reset(SW_Voltage_enabled);
    status_word_mask.set(SW_Quick_stop);
    status_word_mask.set(Switch_On_Disabled);

    homing_mask.set(SW_Target_reached);
    homing_mask.set(SW_Operation_specific0);
    homing_mask.set(SW_Operation_specific1);

    homing_needed_ = false;
    motor_ready_ = false;
    configure_drive_ = false;
    configuring_node_ = true;
  }

  enum StatusWord
  {
    SW_Ready_To_Switch_On=0,
    SW_Switched_On=1,
    SW_Operation_enabled=2,
    SW_Fault=3,
    SW_Voltage_enabled=4,
    SW_Quick_stop=5,
    SW_Switch_on_disabled=6,
    SW_Warning=7,
    SW_Manufacturer_specific0=8,
    SW_Remote=9,
    SW_Target_reached=10,
    SW_Internal_limit=11,
    SW_Operation_specific0=12,
    SW_Operation_specific1=13,
    SW_Manufacturer_specific1=14,
    SW_Manufacturer_specific2=15
  };

  enum ControlWord
  {
    CW_Switch_On=0,
    CW_Enable_Voltage=1,
    CW_Quick_Stop=2,
    CW_Enable_Operation=3,
    CW_Operation_mode_specific0=4,
    CW_Operation_mode_specific1=5,
    CW_Operation_mode_specific2=6,
    CW_Fault_Reset=7,
    CW_Halt=8,
    CW_Reserved0=9,
    CW_Reserved1=10,
    CW_Manufacturer_specific0=11,
    CW_Manufacturer_specific1=12,
    CW_Manufacturer_specific2=13,
    CW_Manufacturer_specific3=14,
    CW_Manufacturer_specific4=15,
  };

  enum OperationMode
  {
    No_Mode = 0,
    Profiled_Position = 1,
    Velocity = 2,
    Profiled_Velocity = 3,
    Profiled_Torque = 4,
    Reserved = 5,
    Homing = 6,
    Interpolated_Position = 7,
    Cyclic_Synchronous_Position = 8,
    Cyclic_Synchronous_Velocity = 9,
    Cyclic_Synchronous_Torque = 10,
  };

  enum SupportedOperationMode
  {
    Sup_Profiled_Position = 0,
    Sup_Velocity = 1,
    Sup_Profiled_Velocity = 2,
    Sup_Profiled_Torque = 3,
    Sup_Reserved = 4,
    Sup_Homing = 5,
    Sup_Interpolated_Position = 6,
    Sup_Cyclic_Synchronous_Position = 7,
    Sup_Cyclic_Synchronous_Velocity = 8,
    Sup_Cyclic_Synchronous_Torque = 9
  };

  enum State
  {
    Start = 0,
    Not_Ready_To_Switch_On = 1,
    Switch_On_Disabled = 2,
    Ready_To_Switch_On = 3,
    Switched_On = 4,
    Operation_Enable = 5,
    Quick_Stop_Active = 6,
    Fault_Reaction_Active = 7,
    Fault = 8,
  };

  const OperationMode getMode();
  bool enterMode(const OperationMode &op_mode);
  bool enterModeAndWait(const OperationMode &op_mode);
  bool isModeSupported(const OperationMode &op_mode);
  static uint32_t getModeMask(const OperationMode &op_mode);
  bool isModeMaskRunning(const uint32_t &mask);

  const State& getState();
  void enterState(const State &s);


  virtual void read(LayerStatus &status);
  virtual void pending(LayerStatus &status);
  virtual void write(LayerStatus &status);

  virtual void diag(LayerReport &report);

  virtual void init(LayerStatus &status);
  virtual void shutdown(LayerStatus &status);

  void getDeviceState(LayerStatus &status);
  void switchMode(LayerStatus &status);

  void motorShutdown();
  void motorSwitchOn();
  void motorSwitchOnandEnableOp();
  void motorDisableVoltage();
  void motorQuickStop();
  void motorDisableOp();
  void motorEnableOp();
  void motorFaultReset();

  virtual void halt(LayerStatus &status);
  virtual void recover(LayerStatus &status);

  const double getActualPos();
  const double getActualInternalPos();

  const double getActualVel();
  const double getActualEff();

  void setTargetPos(const double &target_pos);
  void setTargetVel(const double &target_vel);
  void setTargetEff(const double &v) {}  // TODO(thiagodefreitas)

  const double getTargetPos();
  const double getTargetVel();
  const double getTargetEff()
  {
    return 0;  // TODO(thiagodefreitas)
  }

  bool turnOn();
  bool turnOff();

  void configureEntries();
  void configureModeSpecificEntries();

private:
  boost::shared_ptr <canopen::Node> n_;
  volatile bool running;
  State state_;
  State target_state_;

  bool new_target_pos_;

  bool motor_ready_;
  bool homing_needed_;

  boost::mutex cond_mutex;
  boost::condition_variable cond;

  canopen::ObjectStorage::Entry<canopen::ObjectStorage::DataType<0x006>::type >  status_word;
  canopen::ObjectStorage::Entry<canopen::ObjectStorage::DataType<0x006>::type >  control_word;
  canopen::ObjectStorage::Entry<int8_t>  op_mode_display;
  canopen::ObjectStorage::Entry<int8_t>  op_mode;
  canopen::ObjectStorage::Entry<int16_t>  ip_mode_sub_mode;
  canopen::ObjectStorage::Entry<uint32_t>  supported_drive_modes;

  canopen::ObjectStorage::Entry<int8_t>  homing_method;

  canopen::ObjectStorage::Entry<int32_t> actual_vel;
  canopen::ObjectStorage::Entry<int16_t> target_velocity;
  canopen::ObjectStorage::Entry<uint32_t> profile_velocity;
  canopen::ObjectStorage::Entry<int32_t> actual_pos;
  canopen::ObjectStorage::Entry<int32_t> actual_internal_pos;
  canopen::ObjectStorage::Entry<int32_t> target_position;
  canopen::ObjectStorage::Entry<int32_t> target_interpolated_position;
  canopen::ObjectStorage::Entry<int32_t> target_interpolated_velocity;
  canopen::ObjectStorage::Entry<int32_t> target_profiled_velocity;


  double ac_vel_;
  double ac_eff_;

  OperationMode operation_mode_;
  OperationMode operation_mode_to_set_;
  bool check_mode;

  double ac_pos_;
  double internal_pos_;
  double oldpos_;

  std::bitset<16> status_word_bitset;
  std::bitset<16> control_word_bitset;
  std::bitset<16> status_word_mask;
  std::bitset<16> homing_mask;

  double target_vel_;
  double target_pos_;

  std::vector<int> control_word_buffer;

  void driveSettings();
  void driveSettingsOnlyPos();
  void driveSettingsOnlyBits();

  bool configure_drive_;

  OperationMode default_operation_mode_;

  bool configuring_node_;

  bool recover_active_;
  
  /*template<typename Duration> bool waitMotorReady(const Duration &d){
    time_point t0 = boost::chrono::high_resolution_clock::now() + d;
  
    boost::mutex::scoped_lock cond_lock(cond_mutex);
    motor_ready_ = false;
    while (!motor_ready_)
    {
      if (cond.wait_until(cond_lock, t0) == boost::cv_status::timeout)
      {
          break;
      }
    }
    return motor_ready_;
  }*/
  
};
}  //  namespace canopen
#endif  // CANOPEN_402_CANOPEN_402_H
