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

#ifndef INTERNAL_STATES_H
#define INTERNAL_STATES_H

#include <iostream>
#include <string>
#include <vector>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <canopen_402/enums_402.h>

namespace msm = boost::msm;
namespace mpl = boost::mpl;

namespace canopen
{
// The list of FSM states
struct Not_Ready_To_Switch_On_State : public msm::front::state<>
{
  // every (optional) entry/exit methods get the event passed.
  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {std::cout << "entering: Start" << std::endl;}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {std::cout << "leaving: Start" << std::endl;}
};

struct Switch_On_Disabled_State : public msm::front::state<>
{
  // every (optional) entry/exit methods get the event passed.
  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {std::cout << "entering: Switch_On_Disabled" << std::endl;}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {std::cout << "leaving: Switch_On_Disabled" << std::endl;}
};

struct Ready_To_Switch_On_State : public msm::front::state<>
{
  // every (optional) entry/exit methods get the event passed.
  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {std::cout << "entering: Ready_To_Switch_On" << std::endl;}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& )
  {
    //assert(1==2);
    std::cout << "leaving: Ready_To_Switch_On" << std::endl;
  }
};

struct Switched_On_State : public msm::front::state<>
{
  // every (optional) entry/exit methods get the event passed.
  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {std::cout << "entering: Switched_On" << std::endl;}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {std::cout << "leaving: Switched_On" << std::endl;}
};

struct Operation_Enable_State : public msm::front::state<>
{
  // every (optional) entry/exit methods get the event passed.
  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {std::cout << "entering: Operation_Enable" << std::endl;}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {std::cout << "leaving: Operation_Enable" << std::endl;}
};

struct Quick_Stop_State : public msm::front::state<>
{
  // every (optional) entry/exit methods get the event passed.
  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {std::cout << "entering: Quick_Stop" << std::endl;}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {std::cout << "leaving: Quick_Stop" << std::endl;}
};

struct Fault_State : public msm::front::state<>
{
  // every (optional) entry/exit methods get the event passed.
  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {std::cout << "entering: Fault" << std::endl;}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {std::cout << "leaving: Fault" << std::endl;}
};

// the initial state of the player SM. Must be defined
typedef Not_Ready_To_Switch_On_State initial_state;

}

#endif // INTERNAL_STATES_H
