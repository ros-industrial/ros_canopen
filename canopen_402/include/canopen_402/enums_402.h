/****************************************************************
 *
 * Copyright (c) 2015
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
 * Author: Thiago de Freitas, email:thiagodefreitas@gmail.com
 * Supervised by: Thiago de Freitas, email:thiagodefreitas@gmail.com
 *
 * Date of creation: February 2015
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

#ifndef ENUMS_402_H
#define ENUMS_402_H

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

enum InternalState
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

enum HomingState
{
  Progress = 0,
  NotStarted = 1,
  Attained=2,
  HomingSuccess = 3,
  HomingError = 4,
};

enum InternalActions
{
  ShutdownMotor=0,
  DisableVoltage=1,
  SwitchOn=2,
  EnableOp=3,
  DisableOp=4,
  QuickStop=5,
  FaultReset=6,
  FaultEnable=7,
  DisableQuickStop=8,
};

#endif // ENUMS_402_H
