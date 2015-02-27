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

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <canopen_402/internal_states.h>
#include <bitset>

namespace canopen
{

class MotorSM_ : public msm::front::state_machine_def<MotorSM_>
{
public:
  typedef Not_Ready_To_Switch_On_State initial_state;
  // defined events from transitioning inside the 402 state machine
  struct boot {};
  struct shutdown {};
  struct fault {};
  struct switch_on {};
  struct disable_voltage {};
  struct disable_op {};
  struct enable_op {};
  struct quick_stop {};

  struct newStatusWord {};
  struct newControlWord {};

  // transition actions
  void shutdown_motor(shutdown const&)
  {
    std::cout << "motor_sm::shutdown\n";
    BOOST_THROW_EXCEPTION(std::range_error("Index out of range"));
  }
  void turn_on(switch_on const&)       { std::cout << "motor_sm::switch_on\n"; }
  void turn_off(disable_voltage const&)       { std::cout << "motor_sm::disable_voltage\n"; }
  void activate_QS(quick_stop const&)       { std::cout << "motor_sm::quick_stop\n"; }
  void operate(enable_op const&)       { std::cout << "motor_sm::enable_op\n"; }
  void stop_operation(disable_op const&)       { std::cout << "motor_sm::disable_op\n"; }

  // guard conditions
  bool motor_fault(fault const& evt)
  {
    return false;
  }

  ///////
  /// \brief m
  ///
  ///
  ///
  // the player state machine contains a state which is himself a state machine
  // as you see, no need to declare it anywhere so StatusandControl can be developed separately
  // by another team in another module. For simplicity I just declare it inside player
  struct StatusandControl_ : public msm::front::state_machine_def<StatusandControl_>
  {
    // when StatusandControl, the CD is loaded and we are in either pause or StatusandControl (duh)
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "entering: StatusandControl" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "leaving: StatusandControl" << std::endl;}

    // The list of FSM states
    struct writeControl : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {std::cout << "starting: writeControl" << std::endl;}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {std::cout << "finishing: writeControl" << std::endl;}

    };
    struct readStatus : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {std::cout << "starting: readStatus" << std::endl;}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {std::cout << "finishing: readStatus" << std::endl;}
    };

    // the initial state. Must be defined
    typedef writeControl initial_state;
    // transition actions
    void write_control(newStatusWord const&)       { std::cout << "StatusandControl::write_control\n"; }
    void read_status(newControlWord const&)       { std::cout << "StatusandControl::read_status\n"; }
    // guard conditions

    typedef StatusandControl_ pl; // makes transition table cleaner
    // Transition table for StatusandControl
    struct transition_table : mpl::vector<
        //      Start     Event         Next      Action               Guard
        //    +---------+-------------+---------+---------------------+----------------------+
        a_row < writeControl   , newStatusWord    , readStatus   , &pl::write_control                       >,
        a_row < readStatus   , newControlWord, writeControl   , &pl::read_status                       >
        //    +---------+-------------+---------+---------------------+----------------------+
        > {};
    // Replaces the default no-transition response.
    template <class FSM,class Event>
    void no_transition(Event const& e, FSM&,int state)
    {
      std::cout << "no transition from state " << state
                << " on event " << typeid(e).name() << std::endl;
    }
  };
  // back-end
  typedef msm::back::state_machine<StatusandControl_> StatusandControl;

  /// */


  typedef MotorSM_ m; // makes transition table cleaner

  struct transition_table : mpl::vector<
      //    Start     Event         Next      Action				 Guard
      //  +---------+-------------+---------+---------------------+----------------------+
      _row < Not_Ready_To_Switch_On_State , boot  , Switch_On_Disabled_State                            >,
      //  +---------+-------------+---------+---------------------+----------------------+
      a_row < Switch_On_Disabled_State , shutdown  , Ready_To_Switch_On_State    , &m::shutdown_motor    >,
      g_row < Switch_On_Disabled_State , fault  , Fault_State    ,           &m::motor_fault    >,
      //  +---------+-------------+---------+---------------------+----------------------+
      a_row < Ready_To_Switch_On_State, switch_on , Switched_On_State, &m::turn_on               >,
      a_row < Ready_To_Switch_On_State, disable_voltage , Switch_On_Disabled_State, &m::turn_off               >, //quickstop(?)
      g_row < Ready_To_Switch_On_State , fault  , Fault_State                , &m::motor_fault    >,
      //  +---------+-------------+---------+---------------------+----------------------+
      a_row < Switched_On_State, enable_op , Operation_Enable_State, &m::operate               >,
      a_row < Switched_On_State, shutdown , Ready_To_Switch_On_State, &m::shutdown_motor               >,
      a_row < Switched_On_State, disable_voltage , Switch_On_Disabled_State, &m::turn_off               >, //quickstop(?)
      g_row < Switched_On_State , fault  , Fault_State    ,           &m::motor_fault    >,
      //  +---------+-------------+---------+---------------------+----------------------+
      a_row < Operation_Enable_State, disable_op , Switched_On_State, &m::stop_operation               >,
      a_row < Operation_Enable_State, shutdown , Ready_To_Switch_On_State, &m::shutdown_motor               >,
      a_row < Operation_Enable_State, disable_voltage , Switch_On_Disabled_State, &m::turn_off               >,
      a_row < Operation_Enable_State, quick_stop , Quick_Stop_State, &m::activate_QS               >,
      g_row < Operation_Enable_State , fault  , Fault_State    ,           &m::motor_fault    >,
      //  +---------+-------------+---------+---------------------+----------------------+
      a_row < Quick_Stop_State, enable_op , Operation_Enable_State, &m::operate               >,
      a_row < Quick_Stop_State, disable_voltage , Switch_On_Disabled_State, &m::turn_off               >,
      g_row < Quick_Stop_State , fault  , Fault_State    ,           &m::motor_fault    >
      //  +---------+-------------+---------+---------------------+----------------------+
      //  +---------+-------------+---------+---------------------+----------------------+
      > {};

  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state)
  {
    std::cout << "no transition from state " << state
              << " on event " << typeid(e).name() << std::endl;
  }

  template <class FSM,class Event>
  void exception_caught (Event const&,FSM& fsm,std::exception& )
  {
    std::cout << "Could not switch_state" << std::endl;
    fsm.process_event(disable_voltage());
  }

};


typedef msm::back::state_machine<MotorSM_> motorSM;
};

#endif // STATE_MACHINE_H
