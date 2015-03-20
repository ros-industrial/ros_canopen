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

#ifndef HIGH_LEVEL_SM_H
#define HIGH_LEVEL_SM_H
///////
/// \brief m
///
///
///
// the player state machine contains a state which is himself a state machine
// as you see, no need to declare it anywhere so highLevelSm can be developed separately
// by another team in another module. For simplicity I just declare it inside player
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <canopen_402/enums_402.h>
#include <canopen_402/internal_sm.h>
#include <canopen_402/status_and_control.h>
#include <canopen_402/ip_mode.h>

namespace msm = boost::msm;
namespace mpl = boost::mpl;


using namespace boost::msm::front;

namespace canopen
{
class highLevelSM_ : public msm::front::state_machine_def<highLevelSM_>
{
public:
  highLevelSM_(){}
  highLevelSM_(const boost::shared_ptr<cw_word> &control_word, boost::shared_ptr<double> target_pos
               , boost::shared_ptr<double> target_vel, boost::shared_ptr<OperationMode> operation_mode)
    : control_word_(control_word) , target_vel_(target_vel), target_pos_(target_pos), operation_mode_(operation_mode)
  {
    motorStateMachine = motorSM(control_word_);
    motorStateMachine.start();
    motorStateMachine.process_event(motorSM::boot());

    ipModeMachine = IPMode();
    ipModeMachine.start();
  }
  struct startMachine {};
  struct stopMachine {};

  //  class onSM_ : public msm::front::state_machine_def<onSM_>
  //  {
  //  public:
  //    onSM_() {}
  //    onSM_(const boost::shared_ptr<cw_word> &control_word) : control_word_intern_(control_word)
  //    {
  //      motorStateMachine = motorSM(control_word_intern_);
  //      motorStateMachine.start();
  //      motorStateMachine.process_event(motorSM::boot());
  //    }
  struct enterStandBy {};
  struct checkModeSwitch
  {
    OperationMode op_mode;
    checkModeSwitch() : op_mode() {}
    checkModeSwitch( OperationMode mode) : op_mode(mode) {}
  };
  struct enableMove
  {
    OperationMode op_mode;
    double pos;
    double vel;

    enableMove() : op_mode(OperationMode(0)), pos(0), vel(0) {}
    enableMove( OperationMode mode, double pos, double vel) : op_mode(mode), pos(pos), vel(vel) {}
  };
  struct runMotorSM
  {
    InternalActions action;
    runMotorSM() : action() {}
    runMotorSM( InternalActions actionToTake) : action(actionToTake) {}
  };

  motorSM motorStateMachine;
  IPMode ipModeMachine;
  // when highLevelSm, the CD is loaded and we are in either pause or highLevelSm (duh)
  //      template <class Event,class FSM>
  //      void on_entry(Event const&,FSM& ) {std::cout << "entering: OnSm" << std::endl;}
  //      template <class Event,class FSM>
  //      void on_exit(Event const&,FSM& ) {std::cout << "leaving: OnSm" << std::endl;}

  // The list of FSM states
  struct StartUp : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: StartUp" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: StartUp" << std::endl;}

  };
  // The list of FSM states
  struct Standby : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: CheckUp" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: CheckUp" << std::endl;}

  };

  // The list of FSM states
  struct updateMotorSM : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: updateMotorSM" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: updateMotorSM" << std::endl;}

  };

  struct ModeSwitch : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: ModeSwitch" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: ModeSwitch" << std::endl;}
  };

  struct Move : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: Drive" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: Drive" << std::endl;}
  };

  // the initial state. Must be defined
  // typedef StartUp initial_state;
  // transition actions
  void standby(enterStandBy const&)
  {
    std::cout << "OnSm::standby" << std::endl;
  }

  template <class runMotorSM> void motor_sm(runMotorSM const& evt)
  {
    std::cout << "OnSm::Running motor SM" << evt.action <<std::endl;
    switch(evt.action)
    {
    case FaultReset:
      motorStateMachine.process_event(motorSM::fault_reset());
      std::cout << "Trying to reset the fault" << std::endl;
      break;
    case Shutdown:
      motorStateMachine.process_event(motorSM::shutdown());
      std::cout << "Trying to shutdown" << std::endl;
      break;
    case SwitchOn:
      motorStateMachine.process_event(motorSM::switch_on());
      std::cout << "Trying to switch on" << std::endl;
      break;
    case EnableOp:
      motorStateMachine.process_event(motorSM::enable_op());
      std::cout << "Trying to enable op" << std::endl;
      break;
    default:
      std::cout << "Action not specified" << std::endl;
    }
  }

  template <class checkModeSwitch> void mode_switch(checkModeSwitch const& evt)
  {
    std::cout << "OnSm::switch_mode" << std::endl;
    switch(evt.op_mode)
    {
    case Interpolated_Position:
      ipModeMachine.process_event(IPMode::selectMode());
    }
  }

  template <class enableMove> void move(enableMove const& evt)
  {
    std::cout << "OnSm::drive" << std::endl;
    switch(evt.op_mode)
    {
    case Interpolated_Position:
      ipModeMachine.process_event(IPMode::selectMode());
    }
  }

  // when highLevelSm, the CD is loaded and we are in either pause or highLevelSm (duh)
  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {std::cout << "entering: highLevelSm" << std::endl;}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {std::cout << "leaving: highLevelSm" << std::endl;}

  // The list of FSM states
  struct machineStopped : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: Off" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: Off" << std::endl;}

  };
  struct machineRunning : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: On" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: On" << std::endl;}
  };

  // the initial state. Must be defined
  typedef machineStopped initial_state;
  // transition actions
  void start_machine(startMachine const&)
  {
    std::cout << "highLevelSm::turn_on\n";
  }

  void stop_machine(stopMachine const&)
  {
    std::cout << "highLevelSm::read_status\n";
  }
  // guard conditions

  typedef highLevelSM_ hl; // makes transition table cleaner
  // Transition table for highLevelSm
  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < machineStopped   , startMachine    , machineRunning   , &hl::start_machine                       >,

      Row < machineRunning   , none    , Standby   , none, none                       >,

      a_row < Standby   , runMotorSM    , updateMotorSM   , &hl::motor_sm                       >,
      a_row < Standby   , checkModeSwitch    , ModeSwitch   , &hl::mode_switch                       >,
      a_row < Standby   , enableMove, Move   , &hl::move                     >,
      a_row < Standby   , stopMachine, machineStopped   , &hl::stop_machine                      >,

      a_row < ModeSwitch   , runMotorSM, updateMotorSM   , &hl::motor_sm                     >,
      a_row < ModeSwitch   , enterStandBy, Standby   , &hl::standby                      >,
      a_row < ModeSwitch   , stopMachine, machineStopped   , &hl::stop_machine                      >,

      a_row < updateMotorSM   , checkModeSwitch    , ModeSwitch   , &hl::mode_switch                       >,
      a_row < updateMotorSM   , enterStandBy, Standby   , &hl::standby                      >,
      a_row < updateMotorSM   , enableMove, Move   , &hl::move                     >,
      a_row < updateMotorSM   , stopMachine, machineStopped   , &hl::stop_machine                      >,

      a_row < Move   , enterStandBy, Standby   , &hl::standby                      >,
      a_row < Move   , stopMachine, machineStopped   , &hl::stop_machine                      >
      //    +---------+-------------+---------+---------------------+----------------------+
      > {};
  // Replaces the default no-transition response.
  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state)
  {
    std::cout << "no transition from state " << state
              << " on event " << typeid(e).name() << std::endl;
  }
private:
  boost::shared_ptr<InternalState> state_;
  boost::shared_ptr<InternalState> target_state_;

  boost::shared_ptr<cw_word> control_word_;

  boost::shared_ptr<double> target_pos_;
  boost::shared_ptr<double> target_vel_;
  boost::shared_ptr<OperationMode> operation_mode_;

};
// back-end
typedef msm::back::state_machine<highLevelSM_> highLevelSM;
};

#endif // HIGH_LEVEL_SM_H
