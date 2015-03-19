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

#include <canopen_402/canopen_402.h>

using canopen::Node_402;

void Node_402::pending(LayerStatus &status)
{

}

void Node_402::switchMode(LayerStatus &status)
{

}

bool Node_402::enterMode(const OperationMode &op_mode_var)
{

}

bool Node_402::enterModeAndWait(const OperationMode &op_mode_var)
{
//  motorAbstraction.process_event(highLevelSm::onSM::checkModeSwitch());
}

void Node_402::read(LayerStatus &status)
{
  std::bitset<16> sw_new(7/*status_word.get()*/);
  (*status_word_bitset.get()) = sw_new;

  SwCwSM.process_event(StatusandControl::newStatusWord());

  std::cout << "Read state" << *state_ << std::endl;
  //  operation_mode_ = (OperationMode) op_mode_display.get();
  //  ac_vel_ = actual_vel.get();
  //  ac_pos_ = actual_pos.get();
  //  ac_eff_ = 0; //Currently no effort directly obtained from the HW
}

void Node_402::shutdown(LayerStatus &status)
{
}

void Node_402::diag(LayerReport &report)
{
}

void Node_402::halt(LayerStatus &status)
{

}


void Node_402::recover(LayerStatus &status)
{

}

const double Node_402::getTargetPos()
{
  return target_pos_;
}
const double Node_402::getTargetVel()
{
  return target_vel_;
}


void Node_402::write(LayerStatus &status)
{

//  motorAbstraction.process_event(highLevelSm::onSM::checkUpProcedure());

//  motorAbstraction.process_event(highLevelSm::onSM::enableMove());

  SwCwSM.process_event(StatusandControl::newControlWord());

  int16_t cw_set = static_cast<int>((*control_word_bitset).to_ulong());
 // control_word.set(cw_set);
}


const OperationMode Node_402::getMode()
{
  if(operation_mode_ == Homing) return No_Mode; // TODO: remove after mode switch is handled properly in init
  return operation_mode_;
}

bool Node_402::isModeSupported(const OperationMode &op_mode)
{
  return supported_drive_modes.get_cached() & getModeMask(op_mode);
}
uint32_t Node_402::getModeMask(const OperationMode &op_mode)
{
  switch(op_mode){
  case Profiled_Position:
  case Velocity:
  case Profiled_Velocity:
  case Profiled_Torque:
  case Interpolated_Position:
  case Cyclic_Synchronous_Position:
  case Cyclic_Synchronous_Velocity:
  case Cyclic_Synchronous_Torque:
  case Homing:
    return (1<<(op_mode-1));
  case No_Mode:
    return 0;
  }
  return 0;
}
bool Node_402::isModeMaskRunning(const uint32_t &mask)
{
  return mask & getModeMask(operation_mode_);
}

const double Node_402::getActualVel()
{
  return ac_vel_;
}

const double Node_402::getActualEff()
{
  return ac_eff_;
}

const double Node_402::getActualPos()
{
  return ac_pos_;
}

const double Node_402::getActualInternalPos()
{
  return internal_pos_;
}

void Node_402::setTargetVel(const double &target_vel)
{
  //  if (state_ == Operation_Enable && operation_mode_ == operation_mode_to_set_)
  //  {
  //    target_vel_ = target_vel;
  //  }
}

void Node_402::setTargetPos(const double &target_pos)
{
  //  if (state_ == Operation_Enable && operation_mode_ == operation_mode_to_set_)
  //  {
  //    target_pos_ = target_pos;
  //  }
}

void Node_402::configureEntries()
{
  n_->getStorage()->entry(status_word, 0x6041);
  n_->getStorage()->entry(control_word, 0x6040);

  n_->getStorage()->entry(op_mode, 0x6060);
  n_->getStorage()->entry(op_mode_display, 0x6061);
  n_->getStorage()->entry(supported_drive_modes, 0x6502);

  n_->getStorage()->entry(ip_mode_sub_mode, 0x60C0);

  n_->getStorage()->entry(actual_vel, 0x606C);

  n_->getStorage()->entry(actual_pos, 0x6064);
}

void Node_402::configureModeSpecificEntries()
{
  if (isModeSupported(Profiled_Position))
  {
    n_->getStorage()->entry(target_position, 0x607A);
    n_->getStorage()->entry(profile_velocity, 0x6081);
  }
  if (isModeSupported(Profiled_Velocity))
  {
    n_->getStorage()->entry(target_profiled_velocity, 0x60FF);
  }
  if (isModeSupported(Interpolated_Position))
  {
    n_->getStorage()->entry(target_interpolated_position, 0x60C1, 0x01);
    if (ip_mode_sub_mode.get_cached() == -1)
      n_->getStorage()->entry(target_interpolated_velocity, 0x60C1, 0x02);
  }
  if (isModeSupported(Velocity))
  {
    n_->getStorage()->entry(target_velocity, 0x6042);
  }

  if (isModeSupported(Homing))
  {
    n_->getStorage()->entry(homing_method, 0x6098);
  }
}
//TODO: Implement a smaller state machine for On, Off, Fault, Halt
bool Node_402::turnOn()
{
  motorAbstraction.process_event(highLevelSM::startMachine());

  motorAbstraction.process_event(highLevelSM::runMotorSM(SwitchOn));

  return true;
}

bool Node_402::turnOff()
{
  return true;
}

void Node_402::init(LayerStatus &s)
{
  turnOn();

}
