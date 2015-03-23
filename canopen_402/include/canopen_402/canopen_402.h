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
#include <canopen_402/status_and_control.h>
#include <canopen_402/ip_mode.h>
#include <canopen_402/high_level_sm.h>
#include <string>
#include <vector>

namespace canopen
{
class Node_402 : public canopen::Layer
{
public:
  Node_402(boost::shared_ptr <canopen::Node> n, const std::string &name) : Layer(name), n_(n), check_mode(false)
  {
    operation_mode_ = boost::make_shared<OperationMode>(No_Mode);
    configureEntries();

    homing_mask.set(SW_Target_reached);
    homing_mask.set(SW_Operation_specific0);
    homing_mask.set(SW_Operation_specific1);

    status_word_bitset = boost::make_shared<sw_word>();
    control_word_bitset = boost::make_shared<cw_word>();

    target_pos_ = boost::make_shared<double>();
    target_vel_ = boost::make_shared<double>();

    state_ = boost::make_shared<InternalState>(Start);

    SwCwSM = StatusandControl(status_word_bitset, control_word_bitset, state_);
    motorAbstraction = highLevelSM(control_word_bitset, target_pos_, target_vel_, operation_mode_, state_);
    SwCwSM.start();
    motorAbstraction.start();
    SwCwSM.process_event(StatusandControl::readStatus());
  }

  Node_402(const std::string &name) : Layer(name)
  {
    supported_modes = 255;
    operation_mode_ = boost::make_shared<OperationMode>(No_Mode);
    homing_mask.set(SW_Target_reached);
    homing_mask.set(SW_Operation_specific0);
    homing_mask.set(SW_Operation_specific1);

    target_pos_ = boost::make_shared<double>();
    target_vel_ = boost::make_shared<double>();

    status_word_bitset = boost::make_shared<sw_word>();
    control_word_bitset = boost::make_shared<cw_word>();
    state_ = boost::make_shared<InternalState>(Start);

    SwCwSM = StatusandControl(status_word_bitset, control_word_bitset, state_);
    motorAbstraction = highLevelSM(control_word_bitset, target_pos_, target_vel_, operation_mode_, state_);
    SwCwSM.start();
    motorAbstraction.start();
    SwCwSM.process_event(StatusandControl::readStatus());
  }

  const OperationMode getMode();

  bool enterModeAndWait(const OperationMode &op_mode);
  bool enterModeAndWait(const OperationMode &op_mode, bool wait);
  bool isModeSupported(const OperationMode &op_mode);
  static uint32_t getModeMask(const OperationMode &op_mode);
  bool isModeMaskRunning(const uint32_t &mask);


  virtual void read(LayerStatus &status);
  virtual void pending(LayerStatus &status);
  virtual void write(LayerStatus &status);

  virtual void read();
  virtual void write();

  virtual void processCW(LayerStatus &status);
  virtual void processCW();

  virtual void processSW(LayerStatus &status);
  virtual void processSW();

  virtual void additionalInfo(LayerStatus &status);
  virtual void additionalInfo();

  virtual void diag(LayerReport &report);

  virtual void init(LayerStatus &status);
  virtual void init();
  virtual void shutdown(LayerStatus &status);

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
  bool turnOn(LayerStatus &status);
  bool turnOff(LayerStatus &status);

  void configureEntries();
  void configureModeSpecificEntries();

private:

  boost::shared_ptr <canopen::Node> n_;
  volatile bool running;
  boost::shared_ptr<InternalState> state_;

  bool new_target_pos_;

  bool motor_ready_;
  bool homing_needed_;

  boost::mutex cond_mutex_;
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


  uint32_t supported_modes;
  double ac_vel_;
  double ac_eff_;

  boost::shared_ptr<OperationMode> operation_mode_;
  OperationMode operation_mode_to_set_;
  bool check_mode;

  bool valid_mode_state_;

  double ac_pos_;
  double internal_pos_;
  double oldpos_;

  std::bitset<16> homing_mask;

  boost::shared_ptr<double> target_vel_;
  boost::shared_ptr<double> target_pos_;

  std::vector<int> control_word_buffer;

  bool configure_drive_;

  OperationMode default_operation_mode_;

  bool configuring_node_;

  bool recover_active_;

  bool enter_mode_failure_;

  boost::shared_ptr<canopen::sw_word> status_word_bitset;
  boost::shared_ptr<canopen::cw_word> control_word_bitset;


  StatusandControl SwCwSM;
  highLevelSM motorAbstraction;
};
}  //  namespace canopen
#endif  // CANOPEN_402_CANOPEN_402_H
