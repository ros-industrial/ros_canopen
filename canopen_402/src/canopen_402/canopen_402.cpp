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
  boost::mutex::scoped_lock cond_lock(cond_mutex);

  if(configuring_node_)
  {
    default_operation_mode_ = OperationMode(op_mode.get_cached());
    enterMode(default_operation_mode_);
    configuring_node_ = false;
  }
  else
  {
    control_word_bitset.reset(CW_Halt);

    getDeviceState(status);

    operation_mode_ = (OperationMode) op_mode_display.get();

    if (check_mode)
    {
      switchMode(status);
    }

    if (state_ != target_state_)
    {
      switch (state_)
      {
      case Fault:
        motorFaultReset();
        break;
      case Not_Ready_To_Switch_On:
      case Switch_On_Disabled:
        switch (target_state_)
        {
        case Ready_To_Switch_On:
        case Switched_On:
        case Operation_Enable:
          motorShutdown();
          driveSettingsOnlyPos();
          break;
        }
        break;
      case Ready_To_Switch_On:
        switch (target_state_)
        {
        case Switch_On_Disabled:
          motorDisableVoltage();
          break;
        case Switched_On:
        case Operation_Enable:
          motorSwitchOn();
          driveSettingsOnlyPos();
          break;
        }
        break;
      case Switched_On:
        switch (target_state_)
        {
        case Switch_On_Disabled:
          motorQuickStop();
          break;
        case Ready_To_Switch_On:
          motorShutdown();
          break;
        case Operation_Enable:
          motorEnableOp();
          driveSettingsOnlyPos();
          break;
        }
        break;
      case Operation_Enable:
        switch (target_state_)
        {
        case Switch_On_Disabled:
          motorDisableVoltage();
        case Ready_To_Switch_On:
          motorShutdown();
        case Quick_Stop_Active:
          motorQuickStop();
        case Switched_On:
          motorDisableOp();
        }
        break;
      case Quick_Stop_Active:
        switch (target_state_)
        {
        case Switch_On_Disabled:
          motorDisableVoltage();
          break;
        case Operation_Enable:
          motorEnableOp();
          driveSettingsOnlyPos();
          break;
        }
        break;
      }
    }
    else if (homing_needed_)
    {
      if (operation_mode_ != Homing)
        op_mode.set(Homing);
      else
      {
        control_word_bitset.set(CW_Operation_mode_specific0);

        switch ((status_word_bitset & homing_mask).to_ulong())
        {
        //-------------------------------------------------------------//
        // Op_specific1 | Op_specific0 | Target_reached | Description |
        // ------------ | ------------ | -------------- | ----------- |
        //       0      |       0      |         0      | In Progress |
        //       0      |       0      |         1      | Hom.Started |
        //       0      |       1      |         0      | Hom.Attained, target not reached|
        //       0      |       1      |         1      | Hom.Succesful |
        //       1      |       0      |         0      | Hom.Error, vel!=0 |
        //       1      |       0      |         1      | Hom.Error, vel=0 |
        //       1      |       1      |         0      | Hom.Reserved |
        //       1      |       1      |         1      | Hom.Reserved |
        //-------------------------------------------------------------//
        case 0:
          LOG("Homing in Progress");
          break;
        case (1<<SW_Target_reached):
          LOG("Homing not started");
          break;
        case (1<<SW_Operation_specific0):
          LOG("Homing attained, target not reached");
          break;
        case ((1<<SW_Operation_specific0) | (1<<SW_Target_reached)):
          homing_needed_ = false;
          enterMode(default_operation_mode_);
          LOG("Homing succesful");
          break;
        case (1<<SW_Operation_specific1):
          LOG("Homing error , vel!=0");
          status.error("Homing error, vel!=0");
          break;
        case ((1<<SW_Operation_specific1) | (1<<SW_Target_reached)):
          LOG("Homing error, vel=0");
          status.error("Homing error, vel=0");
          break;
        case (1<<SW_Operation_specific1 | 1<<SW_Operation_specific0):
        case (1<<SW_Operation_specific1 | 1<<SW_Operation_specific0 | 1<<SW_Target_reached):
          LOG("Homing reserved");
          break;
        }
      }
    }
    else if(state_ == Operation_Enable)
    {
      if(configure_drive_)
      {
        ac_pos_ = actual_pos.get();
        ac_vel_ = 0;

        target_pos_ = ac_pos_;
        target_vel_ = ac_vel_;
        configure_drive_ = false;
      }
      else
      {
        driveSettings();
        motor_ready_ = true;
        cond_lock.unlock();
        cond.notify_one();
      }
    }
    int16_t cw_set = static_cast<int>(control_word_bitset.to_ulong());
    control_word.set(cw_set);
  }
}

void Node_402::getDeviceState(LayerStatus &status)
{
  std::bitset<16> sw_new(status_word.get());

  status_word_bitset = sw_new;

  switch ((status_word_bitset & status_word_mask).to_ulong())
  {
  case 0b0000000:
  case 0b0100000:
    state_ = Not_Ready_To_Switch_On;
    break;
  case 0b1000000:
  case 0b1100000:
    state_ = Switch_On_Disabled;
    break;
  case 0b0100001:
    state_ = Ready_To_Switch_On;
    break;
  case 0b0100011:
    state_ = Switched_On;
    break;
  case 0b0100111:
    state_ = Operation_Enable;
    break;
  case 0b0000111:
    state_ = Quick_Stop_Active;
    break;
  case 0b0001111:
  case 0b0101111:
    state_ = Fault_Reaction_Active;
    break;
  case 0b0001000:
  case 0b0101000:
    state_ = Fault;
    break;
  default:
    LOG("Motor currently in an unknown state");
    status.error("Motor currently in an unknown state");
  }
}

void Node_402::switchMode(LayerStatus &status)
{
  target_pos_ = ac_pos_;
  target_vel_ = 0;

  if (operation_mode_ == operation_mode_to_set_)
  {
    if(state_ == Operation_Enable)
    {
        control_word_bitset.reset(CW_Halt);
        check_mode = false;
        motor_ready_ = true;
        cond.notify_all();
    }
    else
    { 
        motorEnableOp();
    }
  }
  else
  {
    op_mode.set(operation_mode_to_set_);
  }
}

bool Node_402::enterMode(const OperationMode &op_mode_var)
{
  control_word_bitset.set(CW_Halt);
  operation_mode_to_set_ = op_mode_var;
  check_mode = true;
  
  target_pos_ = ac_pos_;
  target_vel_ = 0;

  return true;
}

bool Node_402::enterModeAndWait(const OperationMode &op_mode_var)
{
    boost::mutex::scoped_lock cond_lock(cond_mutex);
    
    if(!isModeSupported(op_mode_var)){
      LOG( "Mode " << (int)op_mode_var << " not supported");
      return false;
    }
    
    motor_ready_ = false;
    
    LOG( "Enter mode" << (int)op_mode_var);
    enterMode(op_mode_var);
    time_point t0 = get_abs_time(boost::chrono::seconds(10));
  
    while (!motor_ready_)
    {
      if (cond.wait_until(cond_lock, t0) == boost::cv_status::timeout)
      {
          LOG("Mode Timeout");
          break;
      }
    }
    return motor_ready_;
}

void Node_402::read(LayerStatus &status)
{
  getDeviceState(status);

  operation_mode_ = (OperationMode) op_mode_display.get();
  ac_vel_ = actual_vel.get();
  ac_pos_ = actual_pos.get();
}

void Node_402::shutdown(LayerStatus &status)
{
}

void Node_402::diag(LayerReport &report)
{
}

void Node_402::halt(LayerStatus &status)
{
  //control_word_bitset.set(CW_Halt);
}


void Node_402::recover(LayerStatus &status)
{
  boost::mutex::scoped_lock cond_lock(cond_mutex);

  configure_drive_ = true;
  motor_ready_ = false;
  time_point t0 = boost::chrono::high_resolution_clock::now() + boost::chrono::seconds(10);

  while (!motor_ready_)
  {
    if (cond.wait_until(cond_lock, t0) == boost::cv_status::timeout)
    {
      if (!motor_ready_)
      {
       status.error("Could not properly recover the chain");
       return;
      }
    }
  }
}

const double Node_402::getTargetPos()
{
  return target_pos_;
}
const double Node_402::getTargetVel()
{
  return target_vel_;
}

void Node_402::motorShutdown()
{
  control_word_bitset.reset(CW_Switch_On);
  control_word_bitset.set(CW_Enable_Voltage);
  control_word_bitset.set(CW_Quick_Stop);
  control_word_bitset.reset(CW_Fault_Reset);
  control_word_bitset.reset(CW_Enable_Operation);
  control_word_bitset.reset(CW_Operation_mode_specific0);
  control_word_bitset.reset(CW_Operation_mode_specific1);
  control_word_bitset.reset(CW_Operation_mode_specific2);
}

void Node_402::motorSwitchOn()
{
  control_word_bitset.set(CW_Switch_On);
  control_word_bitset.set(CW_Enable_Voltage);
  control_word_bitset.set(CW_Quick_Stop);
  control_word_bitset.reset(CW_Fault_Reset);
  control_word_bitset.reset(CW_Enable_Operation);
  control_word_bitset.reset(CW_Operation_mode_specific0);
  control_word_bitset.reset(CW_Operation_mode_specific1);
  control_word_bitset.reset(CW_Operation_mode_specific2);
}

void Node_402::motorSwitchOnandEnableOp()
{
  control_word_bitset.set(CW_Switch_On);
  control_word_bitset.set(CW_Enable_Voltage);
  control_word_bitset.set(CW_Quick_Stop);
  control_word_bitset.reset(CW_Fault_Reset);
  control_word_bitset.set(CW_Enable_Operation);
  control_word_bitset.reset(CW_Operation_mode_specific0);
  control_word_bitset.reset(CW_Operation_mode_specific1);
  control_word_bitset.reset(CW_Operation_mode_specific2);
}

void Node_402::motorDisableVoltage()
{
  control_word_bitset.reset(CW_Switch_On);
  control_word_bitset.reset(CW_Enable_Voltage);
  control_word_bitset.reset(CW_Quick_Stop);
  control_word_bitset.reset(CW_Fault_Reset);
  control_word_bitset.reset(CW_Enable_Operation);
  control_word_bitset.reset(CW_Operation_mode_specific0);
  control_word_bitset.reset(CW_Operation_mode_specific1);
  control_word_bitset.reset(CW_Operation_mode_specific2);
}

void Node_402::motorQuickStop()
{
  control_word_bitset.reset(CW_Switch_On);
  control_word_bitset.set(CW_Enable_Voltage);
  control_word_bitset.reset(CW_Quick_Stop);
  control_word_bitset.reset(CW_Fault_Reset);
  control_word_bitset.reset(CW_Enable_Operation);
  control_word_bitset.reset(CW_Operation_mode_specific0);
  control_word_bitset.reset(CW_Operation_mode_specific1);
  control_word_bitset.reset(CW_Operation_mode_specific2);
}

void Node_402::motorDisableOp()
{
  control_word_bitset.set(CW_Switch_On);
  control_word_bitset.set(CW_Enable_Voltage);
  control_word_bitset.set(CW_Quick_Stop);
  control_word_bitset.reset(CW_Fault_Reset);
  control_word_bitset.reset(CW_Enable_Operation);
  control_word_bitset.reset(CW_Operation_mode_specific0);
  control_word_bitset.reset(CW_Operation_mode_specific1);
  control_word_bitset.reset(CW_Operation_mode_specific2);
}

void Node_402::motorEnableOp()
{
  control_word_bitset.set(CW_Switch_On);
  control_word_bitset.set(CW_Enable_Voltage);
  control_word_bitset.set(CW_Quick_Stop);
  control_word_bitset.reset(CW_Fault_Reset);
  control_word_bitset.set(CW_Enable_Operation);
  control_word_bitset.reset(CW_Operation_mode_specific0);
  control_word_bitset.reset(CW_Operation_mode_specific1);
  control_word_bitset.reset(CW_Operation_mode_specific2);
  control_word_bitset.reset(CW_Halt);
}

void Node_402::motorFaultReset()
{
  control_word_bitset.set(CW_Fault_Reset);
  control_word_bitset.reset(CW_Switch_On);
  control_word_bitset.reset(CW_Enable_Voltage);
  control_word_bitset.set(CW_Quick_Stop);
  control_word_bitset.reset(CW_Enable_Operation);
  control_word_bitset.reset(CW_Operation_mode_specific0);
  control_word_bitset.reset(CW_Operation_mode_specific1);
  control_word_bitset.reset(CW_Operation_mode_specific2);
}

void Node_402::driveSettingsOnlyBits()
{
  switch (operation_mode_)
  {
  case Profiled_Position:
    control_word_bitset.reset(CW_Operation_mode_specific0);
    control_word_bitset.reset(CW_Operation_mode_specific1);
    control_word_bitset.reset(CW_Operation_mode_specific2);
    break;
  case Profiled_Velocity:
    control_word_bitset.reset(CW_Operation_mode_specific0);
    control_word_bitset.reset(CW_Operation_mode_specific1);
    control_word_bitset.reset(CW_Operation_mode_specific2);
    break;
  case Interpolated_Position:
    control_word_bitset.set(CW_Operation_mode_specific0);
    control_word_bitset.reset(CW_Operation_mode_specific1);
    control_word_bitset.reset(CW_Operation_mode_specific2);
    break;
  case Velocity:
    control_word_bitset.set(CW_Operation_mode_specific0);
    control_word_bitset.set(CW_Operation_mode_specific1);
    control_word_bitset.set(CW_Operation_mode_specific2);
    break;
  }
}

void Node_402::driveSettingsOnlyPos()
{
  switch (operation_mode_)
  {
  case Profiled_Position:
    target_position.set(target_pos_);
    break;
  case Profiled_Velocity:
    target_profiled_velocity.set(target_vel_);
    break;
  case Interpolated_Position:
    target_interpolated_position.set(target_pos_);
    if (ip_mode_sub_mode.get_cached() == -1)
      target_interpolated_velocity.set(target_vel_);
    break;
  case Velocity:
    target_velocity.set(target_vel_);
    break;
  }
}

void Node_402::driveSettings()
{
  switch (operation_mode_)
  {
  case Profiled_Position:
    if (oldpos_ != target_pos_)
    {
      target_position.set(target_pos_);
      control_word_bitset.set(CW_Operation_mode_specific0);
      control_word_bitset.reset(CW_Operation_mode_specific1);
      control_word_bitset.reset(CW_Operation_mode_specific2);
      oldpos_ = target_pos_;
    }
    else
    {
      control_word_bitset.reset(CW_Operation_mode_specific0);
      control_word_bitset.reset(CW_Operation_mode_specific1);
      control_word_bitset.reset(CW_Operation_mode_specific2);
    }
    break;
  case Profiled_Velocity:
    target_profiled_velocity.set(target_vel_);
    control_word_bitset.reset(CW_Operation_mode_specific0);
    control_word_bitset.reset(CW_Operation_mode_specific1);
    control_word_bitset.reset(CW_Operation_mode_specific2);
    break;
  case Interpolated_Position:
    if (oldpos_ != target_pos_)
    {
      target_interpolated_position.set(target_pos_);
      if (ip_mode_sub_mode.get_cached() == -1)
        target_interpolated_velocity.set(target_vel_);
      control_word_bitset.set(CW_Operation_mode_specific0);
      control_word_bitset.reset(CW_Operation_mode_specific1);
      control_word_bitset.reset(CW_Operation_mode_specific2);
      oldpos_ = target_pos_;
    }
    else
    {
    //  control_word_bitset.reset(CW_Operation_mode_specific0);
      control_word_bitset.reset(CW_Operation_mode_specific1);
      control_word_bitset.reset(CW_Operation_mode_specific2);
    }
    break;
  case Velocity:
    target_velocity.set(target_vel_);
    control_word_bitset.set(CW_Operation_mode_specific0);
    control_word_bitset.set(CW_Operation_mode_specific1);
    control_word_bitset.set(CW_Operation_mode_specific2);
    break;
  }
}
void Node_402::write(LayerStatus &status)
{
  if (check_mode)
  {
    switchMode(status);
  }

  else if (state_ == Operation_Enable)
  {
    driveSettings();
  }
  else
    status.error("Motor not in operation enabled state");

  int16_t cw_set = static_cast<int>(control_word_bitset.to_ulong());
  control_word.set(cw_set);
}

const Node_402::State& Node_402::getState()
{
  return state_;
}

const Node_402::OperationMode Node_402::getMode()
{
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
  if (state_ == Operation_Enable && operation_mode_ == operation_mode_to_set_)
  {
    target_vel_ = target_vel;
  }
}

void Node_402::setTargetPos(const double &target_pos)
{
  if (state_ == Operation_Enable && operation_mode_ == operation_mode_to_set_)
  {
    target_pos_ = target_pos;
  }
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

bool Node_402::turnOn()
{
  target_state_ = Operation_Enable;
  return true;
}

bool Node_402::turnOff()
{
  target_state_ = Switch_On_Disabled;
  return true;
}

void Node_402::init(LayerStatus &s)
{
  boost::mutex::scoped_lock cond_lock(cond_mutex);

  motor_ready_ = false;
  configure_drive_ = true;

  time_point t0 = boost::chrono::high_resolution_clock::now() + boost::chrono::seconds(10);

  Node_402::configureModeSpecificEntries();

  if (homing_method.get() != 0)
    homing_needed_ = true;

  if (Node_402::turnOn())
  {
    running = true;
  }
  else
    s.error();

  while (!motor_ready_)
  {
    if (cond.wait_until(cond_lock, t0) == boost::cv_status::timeout)
    {
      if (!motor_ready_)
      {
        s.error("Could not properly initialize the chain");
        return;
      }
    }
  }
  boost::this_thread::sleep_for(boost::chrono::milliseconds(10));

  if(state_!=Operation_Enable)
  {
    if(status_word_bitset.test(SW_Quick_stop))
    {
      motor_ready_ = false;

      time_point t0 = boost::chrono::high_resolution_clock::now() + boost::chrono::seconds(10);

      while (!motor_ready_)
      {
        if (cond.wait_until(cond_lock, t0) == boost::cv_status::timeout)
        {
          if (!motor_ready_)
          {
              s.error("Could not properly initialize the chain");
              return;
          }
        }
      }
    }


    else
    {
      s.error("Could not properly initialize the chain");
      return;
      }
  }
  else
    LOG("Propertly initialized module");
}
