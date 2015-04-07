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
  processSW(status);
  processCW(status);
  additionalInfo(status);

  clearTargetEntries();
}

bool Node_402::enterModeAndWait(const OperationMode &op_mode_var)
{
  boost::mutex::scoped_lock lock(mode_mutex_, boost::try_to_lock);
  if(!lock) return false;

  motorEvent(highLevelSM::enterStandBy());

  canopen::time_point abs_time = canopen::get_abs_time(boost::chrono::seconds(1));
  canopen::time_point actual_point;

  valid_mode_state_ = false;

  if (isModeSupported(op_mode_var))
  {
    op_mode.set_cached(op_mode_var);

    bool transition_success = motorEvent(highLevelSM::checkModeSwitch(op_mode_var));

    while(transition_success == boost::msm::back::HANDLED_FALSE)
    {
      actual_point = boost::chrono::high_resolution_clock::now();
      if(boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
      {
        return false;
      }
      transition_success = motorEvent(highLevelSM::checkModeSwitch(op_mode_var));

      motorEvent(highLevelSM::enterStandBy());
      //return false;
    }
    valid_mode_state_ = true;
    return true;
  }
  else
    return false;
}


void Node_402::processSW(LayerStatus &status)
{
  boost::mutex::scoped_lock lock(word_mutex_, boost::try_to_lock);
  if(!lock) return;

  std::bitset<16> sw_new(status_word.get());

  *status_word_bitset = sw_new;

  SwCwSM.process_event(StatusandControl::newStatusWord());

  //  if(*state_ == Fault)
  //  {
  //    bool transition_success;
  //    transition_success =  motorEvent(highLevelSM::runMotorSM(FaultEnable)); //this is the timeout in milliseconds
  //  }
}

void Node_402::additionalInfo(LayerStatus &s)
{

  boost::mutex::scoped_lock lock(word_mutex_, boost::try_to_lock);
  if(!lock) return;

  *operation_mode_ = (OperationMode) op_mode_display.get();

  ac_vel_ = actual_vel.get();
  ac_pos_ = actual_pos.get();
  ac_eff_ = 0; //Currently,no effort value is directly obtained from the HW
}

void Node_402::handleRead(LayerStatus &status, const LayerState &current_state)
{
  if(current_state == Init || current_state == Recover)
  {
    pending(status);
  }
  processSW(status);
  additionalInfo(status);
}

void Node_402::handleWrite(LayerStatus &status, const LayerState &current_state)
{
  if(current_state == Init || current_state == Recover)
  {
    return;
  }
  move(status);
  processCW(status);
  motorEvent(highLevelSM::enterStandBy());
}

void Node_402::processCW(LayerStatus &status)
{
  boost::mutex::scoped_lock lock(word_mutex_, boost::try_to_lock);
  if(!lock) return;

  int16_t cw_set = static_cast<int>((*control_word_bitset).to_ulong());

  control_word.set(cw_set);

  SwCwSM.process_event(StatusandControl::newControlWord());
}

void Node_402::move(LayerStatus &status)
{
  if(*state_ == Operation_Enable)
  {
    bool transition_success = motorEvent(highLevelSM::enableMove(*operation_mode_, (*target_values_).target_pos, (*target_values_).target_vel));
    if(transition_success)
    {
      target_interpolated_position.set((*target_values_).target_pos);
      if (ip_mode_sub_mode.get_cached() == -1)
        target_interpolated_velocity.set((*target_values_).target_vel);
    }
  }
}

void Node_402::handleShutdown(LayerStatus &status)
{
  turnOff(status);
}

void Node_402::handleDiag(LayerReport &report)
{
}

void Node_402::handleHalt(LayerStatus &status)
{
  bool transition_success;

  transition_success = motorEvent(highLevelSM::runMotorSM(QuickStop));

  //  if(!transition_success)
  //    status.error("Could not halt the module");

  motorEvent(highLevelSM::enterStandBy());
}


void Node_402::handleRecover(LayerStatus &status)
{
  bool recover_success;

  motorEvent(highLevelSM::stopMachine());

  recover_success = turnOn(status);

  if(!recover_success)
    status.error("Failed to Recover the modules");
}

const double Node_402::getTargetPos()
{
  return (*target_values_).target_pos;
}
const double Node_402::getTargetVel()
{
  return (*target_values_).target_vel;
}


const OperationMode Node_402::getMode()
{
  if(*operation_mode_ == Homing) return No_Mode; // TODO: remove after mode switch is handled properly in init
  return *operation_mode_;
}

bool Node_402::isModeSupported(const OperationMode &op_mode)
{
  return supported_modes & getModeMask(op_mode);
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
  return mask & getModeMask(*operation_mode_);
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
  if (*state_ == Operation_Enable)
  {
    (*target_values_).target_vel = target_vel;
  }
  else
    (*target_values_).target_vel = 0;
}

void Node_402::setTargetPos(const double &target_pos)
{
  if (*state_ == Operation_Enable)
  {
    (*target_values_).target_pos = target_pos;
  }
  else
    (*target_values_).target_pos = ac_pos_;
}

void Node_402::configureEntries()
{
  n_->getStorage()->entry(status_word, 0x6041);
  n_->getStorage()->entry(control_word, 0x6040);

  n_->getStorage()->entry(op_mode, 0x6060);
  n_->getStorage()->entry(op_mode_display, 0x6061);
  n_->getStorage()->entry(supported_drive_modes, 0x6502);

  n_->getStorage()->entry(actual_vel, 0x606C);

  n_->getStorage()->entry(actual_pos, 0x6064);
}

void Node_402::configureModeSpecificEntries()
{
  supported_modes = supported_drive_modes.get_cached();

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
    n_->getStorage()->entry(ip_mode_sub_mode, 0x60C0);
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
void Node_402::clearTargetEntries()
{
  (*target_values_).target_pos = ac_pos_;
  (*target_values_).target_vel = 0;

  target_interpolated_position.set((*target_values_).target_pos);
  if (ip_mode_sub_mode.get_cached() == -1)
    target_interpolated_velocity.set((*target_values_).target_vel);
}

//TODO: Implement a smaller state machine for On, Off, Fault, Halt
bool Node_402::turnOn(LayerStatus &s)
{
  boost::mutex::scoped_lock cond_lock(cond_mutex_);
  if(!cond_lock)
    return false;

  clearTargetEntries();

  canopen::time_point abs_time = canopen::get_abs_time(boost::chrono::seconds(2));
  canopen::time_point actual_point;

  bool transition_success;

  motorEvent(highLevelSM::startMachine());

  while(*state_ == Start)
  {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }

  if(*state_ == Fault)
  {
    transition_success =  motorEvent(highLevelSM::runMotorSM(FaultEnable)); //this is the timeout in milliseconds
    motorEvent(highLevelSM::enterStandBy());
    if(!transition_success)
    {
      s.error("Could not properly set the device to a fault state");
      return false;
    }

    transition_success =  motorEvent(highLevelSM::runMotorSM(FaultReset)); //this is the timeout in milliseconds

    motorEvent(highLevelSM::enterStandBy());


    while(!transition_success)
    {
      actual_point = boost::chrono::high_resolution_clock::now();
      if(boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
      {
        s.error("Could not reset fault");
        return false;
      }

      transition_success =  motorEvent(highLevelSM::runMotorSM(FaultReset)); //this is the timeout in milliseconds

      motorEvent(highLevelSM::enterStandBy());
    }

    motorEvent(highLevelSM::enterStandBy());
  }


  if(*state_==Quick_Stop_Active)
  {
    transition_success = motorEvent(highLevelSM::runMotorSM(DisableQuickStop));
    motorEvent(highLevelSM::enterStandBy());

    while(!transition_success)
    {
      actual_point = boost::chrono::high_resolution_clock::now();
      if( boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
      {
        s.error("Could not disable the quick stop");
        return false;
      }
      transition_success = motorEvent(highLevelSM::runMotorSM(DisableQuickStop));
      motorEvent(highLevelSM::enterStandBy());
      //return false;
    }
    motorEvent(highLevelSM::enterStandBy());
  }


  boost::this_thread::sleep_for(boost::chrono::milliseconds(10));

  transition_success = motorEvent(highLevelSM::runMotorSM(ShutdownMotor));
  motorEvent(highLevelSM::enterStandBy());

  while(!transition_success)
  {
    actual_point = boost::chrono::high_resolution_clock::now();
    if( boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
    {
      s.error("Could not prepare the device");
      return false;
    }
    transition_success = motorEvent(highLevelSM::runMotorSM(ShutdownMotor));
    motorEvent(highLevelSM::enterStandBy());
  }
  motorEvent(highLevelSM::enterStandBy());

  transition_success = motorEvent(highLevelSM::runMotorSM(SwitchOn));
  motorEvent(highLevelSM::enterStandBy());


  while(!transition_success)
  {
    actual_point = boost::chrono::high_resolution_clock::now();
    if(boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
    {
      s.error("Could not switch on");
      return false;
    }
    transition_success = motorEvent(highLevelSM::runMotorSM(SwitchOn));
    motorEvent(highLevelSM::enterStandBy());
  }
  motorEvent(highLevelSM::enterStandBy());

  transition_success = motorEvent(highLevelSM::runMotorSM(EnableOp));
  motorEvent(highLevelSM::enterStandBy());

  while(!transition_success)
  {
    actual_point = boost::chrono::high_resolution_clock::now();
    if(boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
    {
      s.error("Could not enable op");
      return false;
    }
    transition_success = motorEvent(highLevelSM::runMotorSM(EnableOp));
    motorEvent(highLevelSM::enterStandBy());
    //return false;
  }
  motorEvent(highLevelSM::enterStandBy());

  return true;
}

template<class Event>
bool Node_402::motorEvent(Event const& evt)
{
  boost::mutex::scoped_lock lock(motor_mutex_, boost::try_to_lock);
  if(!lock) return false;

  bool transition_success;

  transition_success = motorAbstraction.process_event(evt);

  return transition_success;
}

bool Node_402::turnOff(LayerStatus &s)
{
  motorEvent(highLevelSM::runMotorSM(ShutdownMotor));

  motorEvent(highLevelSM::stopMachine());


  return true;
}


void Node_402::handleInit(LayerStatus &s)
{
  Node_402::configureModeSpecificEntries();

  bool turn_on = Node_402::turnOn(s);

  if (turn_on)
  {
    if (homing_method.valid() && homing_method.get() != 0)
    {
      bool transition_success;

      transition_success = enterModeAndWait(Homing);
      if(!transition_success)
      {
        s.error("Failed to do the homing procedure");
      }
    }
  }
  else
    s.error("Could not properly initialize the module");
}
