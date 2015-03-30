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

#ifndef STATUS_AND_CONTROL_H
#define STATUS_AND_CONTROL_H

#include <string>
#include <vector>
#include <canopen_402/enums_402.h>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>

namespace msm = boost::msm;
namespace mpl = boost::mpl;

using namespace boost::msm::front;
///////
/// \brief m
///
///
///
// the status and control state machine
namespace canopen
{
typedef std::bitset<16> sw_word;
typedef std::bitset<16> cw_word;

class StatusandControl_ : public msm::front::state_machine_def<StatusandControl_>
{
public:
  StatusandControl_(){}
  StatusandControl_(boost::shared_ptr<sw_word> status_word, boost::shared_ptr<cw_word> control_word, const boost::shared_ptr<InternalState> &state) : status_word_(status_word), control_word_(control_word), state_(state)
  {
    status_word_mask_.set(SW_Ready_To_Switch_On);
    status_word_mask_.set(SW_Switched_On);
    status_word_mask_.set(SW_Operation_enabled);
    status_word_mask_.set(SW_Fault);
    status_word_mask_.reset(SW_Voltage_enabled);
    status_word_mask_.set(SW_Quick_stop);
    status_word_mask_.set(Switch_On_Disabled);
  }

  struct commandTargets
  {
    double target_pos;
    double target_vel;
    commandTargets() : target_pos(0), target_vel(0) {}
    commandTargets(double pos) : target_pos(pos), target_vel(0) {}
    commandTargets(double pos, double vel) : target_pos(pos), target_vel(vel) {}
  };


  struct newStatusWord {};
  struct newControlWord {};

  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {/*std::cout << "entering: StatusandControl" << std::endl;*/}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: StatusandControl" << std::endl;*/}

  // The list of FSM states
  struct writeControl : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
      // std::cout << "starting: writeControl" << std::endl;
    }
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
      //std::cout << "finishing: writeControl" << std::endl;
    }

  };
  struct readStatus : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
      //std::cout << "starting: readStatus" << std::endl;
    }
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
      //std::cout << "finishing: readStatus" << std::endl;
    }
  };

  // the initial state. Must be defined
  typedef writeControl initial_state;
  // transition actions
  void write_control(newStatusWord const&)
  {
    // std::cout << "StatusandControl::write_control\n";
    //std::cout << "STATUS_WRITE:" << (*status_word_.get()) << std::endl;
    //std::cout << "CONTROL_WRITE:" << (*control_word_.get()) << std::endl;
  }

  boost::shared_ptr<InternalState> getState()
  {
    return state_;
  }

  void read_status(newControlWord const&)
  {
    switch (((*status_word_.get()) & status_word_mask_).to_ulong())
    {
    case 0b0000000:
    case 0b0100000:
      *state_ = Not_Ready_To_Switch_On;
      break;
    case 0b1000000:
    case 0b1100000:
      *state_ = Switch_On_Disabled;
      break;
    case 0b0100001:
      *state_ = Ready_To_Switch_On;
      break;
    case 0b0100011:
      *state_ = Switched_On;
      break;
    case 0b0100111:
      *state_ = Operation_Enable;
      break;
    case 0b0000111:
      *state_ = Quick_Stop_Active;
      break;
    case 0b0001111:
    case 0b0101111:
      *state_ = Fault_Reaction_Active;
      break;
    case 0b0001000:
    case 0b0101000:
      *state_ = Fault;
      break;
    default:
      LOG("Motor currently in an unknown state");
    }
  }
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
    //    std::cout << "no transition from state " << state
    //              << " on event " << typeid(e).name() << std::endl;
  }

private:
  boost::shared_ptr<sw_word> status_word_;
  boost::shared_ptr<cw_word> control_word_;
  std::bitset<16> status_word_mask_;
  boost::shared_ptr<InternalState> state_;
};
// back-end
typedef msm::back::state_machine<StatusandControl_> StatusandControl;
};
/// */
///
#endif // STATUS_AND_CONTROL_H
