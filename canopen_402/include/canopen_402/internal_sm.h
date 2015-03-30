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
 * Date of creation: March 2015
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

#include <canopen_402/status_and_control.h>
#include <bitset>

namespace canopen
{

class MotorSM_ : public msm::front::state_machine_def<MotorSM_>
{
public:
  MotorSM_(){}
  MotorSM_(const boost::shared_ptr<cw_word> &control_word, boost::shared_ptr<InternalState> actual_state) : control_word_(control_word), state_(actual_state)
  {
     target_state_ = boost::make_shared<InternalState>();
  }

  // defined events from transitioning inside the 402 state machine
  struct boot {};
  struct shutdown {};
  struct fault {};
  struct switch_on {};
  struct disable_voltage {};
  struct disable_op {};
  struct enable_op {};
  struct quick_stop {};
  struct fault_reset {};

  struct Not_Ready_To_Switch_On_State : public msm::front::state<>
  {
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "entering: Not_Ready_To_Switch_On_State" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: Not_Ready_To_Switch_On_State" << std::endl;*/}
  };

  struct Switch_On_Disabled_State : public msm::front::state<>
  {
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "entering: Switch_On_Disabled" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: Switch_On_Disabled" << std::endl;*/}
  };

  struct Ready_To_Switch_On_State : public msm::front::state<>
  {
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "entering: Ready_To_Switch_On" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
//      std::cout << "leaving: Ready_To_Switch_On" << std::endl;
    }
  };

  struct Switched_On_State : public msm::front::state<>
  {
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "entering: Switched_On" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: Switched_On" << std::endl;*/}
  };

  struct Operation_Enable_State : public msm::front::state<>
  {
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "entering: Operation_Enable" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: Operation_Enable" << std::endl;*/}
  };

  struct Quick_Stop_State : public msm::front::state<>
  {
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "entering: Quick_Stop" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: Quick_Stop" << std::endl;*/}
  };

  struct Fault_State : public msm::front::state<>
  {
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "entering: Fault" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: Fault" << std::endl;*/}
  };

  // the initial state of the player SM. Must be defined
  typedef Not_Ready_To_Switch_On_State initial_state;
  // transition actions
  void shutdown_motor(shutdown const&)
  {
    control_word_->reset(CW_Switch_On);
    control_word_->set(CW_Enable_Voltage);
    control_word_->set(CW_Quick_Stop);
    control_word_->reset(CW_Fault_Reset);
    control_word_->reset(CW_Enable_Operation);
    control_word_->reset(CW_Operation_mode_specific0);
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
  }
  void turn_on(switch_on const&)
  {
    control_word_->set(CW_Switch_On);
    control_word_->set(CW_Enable_Voltage);
    control_word_->set(CW_Quick_Stop);
    control_word_->reset(CW_Fault_Reset);
    control_word_->reset(CW_Enable_Operation);
    control_word_->reset(CW_Operation_mode_specific0);
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
  }
  void turn_off(disable_voltage const&)
  {
    control_word_->reset(CW_Switch_On);
    control_word_->reset(CW_Enable_Voltage);
    control_word_->set(CW_Quick_Stop);
    control_word_->reset(CW_Fault_Reset);
    control_word_->reset(CW_Enable_Operation);
    control_word_->reset(CW_Operation_mode_specific0);
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
  }
  void activate_QS(quick_stop const&)
  {
    control_word_->reset(CW_Switch_On);
    control_word_->set(CW_Enable_Voltage);
    control_word_->reset(CW_Quick_Stop);
    control_word_->reset(CW_Fault_Reset);
    control_word_->reset(CW_Enable_Operation);
    control_word_->reset(CW_Operation_mode_specific0);
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
  }
  void operate(enable_op const&)
  {
    control_word_->set(CW_Switch_On);
    control_word_->set(CW_Enable_Voltage);
    control_word_->set(CW_Quick_Stop);
    control_word_->reset(CW_Fault_Reset);
    control_word_->set(CW_Enable_Operation);
    control_word_->reset(CW_Operation_mode_specific0);
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
  }
  void stop_operation(disable_op const&)
  {
    control_word_->set(CW_Switch_On);
    control_word_->set(CW_Enable_Voltage);
    control_word_->set(CW_Quick_Stop);
    control_word_->reset(CW_Fault_Reset);
    control_word_->reset(CW_Enable_Operation);
    control_word_->reset(CW_Operation_mode_specific0);
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
  }

  void reset_fault(fault_reset const&)
  {
    control_word_->set(CW_Fault_Reset);
    control_word_->reset(CW_Switch_On);
    control_word_->reset(CW_Enable_Voltage);
    control_word_->set(CW_Quick_Stop);
    control_word_->reset(CW_Enable_Operation);
    control_word_->reset(CW_Operation_mode_specific0);
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
  }

  // guard conditions
  bool motor_fault(fault const& evt)
  {
    return true;
  }

  typedef MotorSM_ m; // makes transition table cleaner

  struct transition_table : mpl::vector<
      //    Start     Event         Next      Action				 Guard
      //  +---------+-------------+---------+---------------------+----------------------+
      Row < Not_Ready_To_Switch_On_State , none  , Switch_On_Disabled_State, none                            >,
      //  +---------+-------------+---------+---------------------+----------------------+
      a_row < Switch_On_Disabled_State , shutdown  , Ready_To_Switch_On_State    , &m::shutdown_motor    >,
      Row < Switch_On_Disabled_State , fault  , Fault_State    , none, none              >,
      //  +---------+-------------+---------+---------------------+----------------------+
      a_row < Ready_To_Switch_On_State, switch_on , Switched_On_State, &m::turn_on              >,
      a_row < Ready_To_Switch_On_State, disable_voltage , Switch_On_Disabled_State, &m::turn_off               >, //quickstop(?)
      Row < Ready_To_Switch_On_State , fault  , Fault_State    , none, none              >,
      //  +---------+-------------+---------+---------------------+----------------------+
      a_row < Switched_On_State, enable_op , Operation_Enable_State, &m::operate               >,
      a_row < Switched_On_State, shutdown , Ready_To_Switch_On_State, &m::shutdown_motor               >,
      a_row < Switched_On_State, disable_voltage , Switch_On_Disabled_State, &m::turn_off               >, //quickstop(?)
      Row < Switched_On_State , fault  , Fault_State    , none, none              >,
      //  +---------+-------------+---------+---------------------+----------------------+
      a_row < Operation_Enable_State, disable_op , Switched_On_State, &m::stop_operation               >,
      a_row < Operation_Enable_State, shutdown , Ready_To_Switch_On_State, &m::shutdown_motor               >,
      a_row < Operation_Enable_State, disable_voltage , Switch_On_Disabled_State, &m::turn_off               >,
      a_row < Operation_Enable_State, quick_stop , Quick_Stop_State, &m::activate_QS               >,
      Row < Operation_Enable_State , fault  , Fault_State    , none, none              >,
      //  +---------+-------------+---------+---------------------+----------------------+
      a_row < Quick_Stop_State, enable_op , Operation_Enable_State, &m::operate               >,
      a_row < Quick_Stop_State, disable_voltage , Ready_To_Switch_On_State, &m::turn_off               >,
      Row < Quick_Stop_State , fault  , Fault_State    , none, none              >,

      a_row < Fault_State , fault_reset  , Switch_On_Disabled_State    , &m::reset_fault    >
      //  +---------+-------------+---------+---------------------+----------------------+
      //  +---------+-------------+---------+---------------------+----------------------+
      > {};

  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state)
  {
  // std::cout << "no transition from state " << state
   //           << " on event " << typeid(e).name() << std::endl;
  }

  template <class FSM,class Event>
  void exception_caught (Event const&,FSM& fsm,std::exception& )
  {
//    std::cout << "Could not switch_state" << std::endl;
    //fsm.process_event(disable_voltage());
  }
private:
  boost::shared_ptr<cw_word> control_word_;
  boost::shared_ptr<InternalState> state_;
  boost::shared_ptr<InternalState> target_state_;
};


typedef msm::back::state_machine<MotorSM_> motorSM;
};

#endif // STATE_MACHINE_H
