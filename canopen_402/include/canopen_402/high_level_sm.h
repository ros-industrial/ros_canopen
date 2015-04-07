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
// the high level state machine
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <canopen_402/enums_402.h>
#include <canopen_402/internal_sm.h>
#include <canopen_402/status_and_control.h>
#include <canopen_402/ip_mode.h>
#include <canopen_402/homing.h>
#include <boost/thread/thread.hpp>

namespace msm = boost::msm;
namespace mpl = boost::mpl;


using namespace boost::msm::front;

namespace canopen
{
class highLevelSM_ : public msm::front::state_machine_def<highLevelSM_>
{
public:
  highLevelSM_(){}
  highLevelSM_(const boost::shared_ptr<cw_word> &control_word,const boost::shared_ptr<sw_word> &status_word,  boost::shared_ptr<StatusandControl::commandTargets> target, boost::shared_ptr<OperationMode> operation_mode, const boost::shared_ptr<InternalState> &actual_state)
    : control_word_(control_word) , status_word_(status_word), targets_(target), operation_mode_(operation_mode), state_(actual_state), previous_mode_(No_Mode)
  {
    motorStateMachine = motorSM(control_word_, state_);
    motorStateMachine.start();
    motorStateMachine.process_event(motorSM::boot());

    homingMachine = HomingSM(control_word_, status_word_);
    homingMachine.start();

    ipModeMachine = IPMode(control_word_);
    ipModeMachine.start();
  }
  struct startMachine {};
  struct stopMachine {};

  struct enterStandBy {};

  struct updateSwitch {};
  struct updateMotor {};

  struct checkModeSwitch
  {
    OperationMode op_mode;
    int timeout;

    checkModeSwitch() : op_mode(), timeout(0) {}
    checkModeSwitch( OperationMode mode) : op_mode(mode), timeout(0) {}
    checkModeSwitch( OperationMode mode, int timeOut) : op_mode(mode), timeout(timeOut) {}
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
    int timeout;

    runMotorSM() : action(), timeout(0) {}
    runMotorSM(InternalActions actionToTake) : action(actionToTake), timeout(0) {}
    runMotorSM(InternalActions actionToTake, int timeOut) : action(actionToTake), timeout(timeOut) {}
  };

  motorSM motorStateMachine;
  IPMode ipModeMachine;
  HomingSM homingMachine;

  // The list of FSM states
  struct StartUp : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: StartUp" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: StartUp" << std::endl;*/}

  };
  // The list of FSM states
  struct Standby : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: Standby" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: Standby" << std::endl;*/}

  };

  // The list of FSM states
  struct updateMotorSM : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: updateMotorSM" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: updateMotorSM" << std::endl;*/}

  };

  struct ModeSwitch : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: ModeSwitch" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: ModeSwitch" << std::endl;*/}
  };

  struct Move : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: Move" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {/*std::cout << "finishing: Move" << std::endl;*/}
  };

  // the initial state. Must be defined
  // typedef StartUp initial_state;
  // transition actions
  void standby(enterStandBy const&)
  {
    //    std::cout << "OnSm::standby" << std::endl;
  }


  template <class runMotorSM> void motor_sm(runMotorSM const& evt)
  {
    switch(evt.action)
    {
    case QuickStop:
      ipModeMachine.process_event(IPMode::deselectMode());
      motorStateMachine.process_event(motorSM::quick_stop());
      if(*state_ != Quick_Stop_Active)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case FaultReset:
      motorStateMachine.process_event(motorSM::fault_reset());
      if(*state_ == Fault)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case ShutdownMotor:
      motorStateMachine.process_event(motorSM::shutdown());
      if(*state_ != Ready_To_Switch_On)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case SwitchOn:
      motorStateMachine.process_event(motorSM::switch_on());
      if(*state_ != Switched_On)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case EnableOp:
      motorStateMachine.process_event(motorSM::enable_op());
      if(*state_ != Operation_Enable)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case FaultEnable:
      ipModeMachine.process_event(IPMode::deselectMode());
      motorStateMachine.process_event(motorSM::fault());
      if(*state_ != Fault)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case DisableQuickStop:
      motorStateMachine.process_event(motorSM::disable_voltage());
      if(*state_ != Not_Ready_To_Switch_On && *state_ != Switch_On_Disabled)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    default:
      std::cout << "Action not specified" << std::endl;
    }
  }

  template <class checkModeSwitch> void mode_switch(checkModeSwitch const& evt)
  {
    if(*operation_mode_ != evt.op_mode)
    {
      BOOST_THROW_EXCEPTION(std::invalid_argument("This operation mode can not be used"));
      previous_mode_ = *operation_mode_;
    }

    switch(evt.op_mode)
    {
    case Interpolated_Position:
      ipModeMachine.process_event(IPMode::selectMode());
      break;
    case Homing:
      homingMachine.process_event(HomingSM::selectMode());
      bool transition_sucess = homingMachine.process_event(HomingSM::runHomingCheck());
      if(!transition_sucess)
      {
        BOOST_THROW_EXCEPTION(std::invalid_argument("Homing still not completed"));
      }
      break;
    }
  }

  template <class enableMove> void move(enableMove const& evt)
  {
    //    std::cout << "OnSm::drive" << std::endl;
    switch(evt.op_mode)
    {
    case Interpolated_Position:
      ipModeMachine.process_event(IPMode::selectMode());

      ipModeMachine.process_event(IPMode::enableIP());
    }
  }

  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {/*std::cout << "entering: highLevelSm" << std::endl;*/}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: highLevelSm" << std::endl;*/}

  // The list of FSM states
  struct machineStopped : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: Off" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: Off" << std::endl;*/}

  };
  struct machineRunning : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: On" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: On" << std::endl;*/}
  };

  // the initial state. Must be defined
  typedef machineStopped initial_state;
  // transition actions
  void start_machine(startMachine const&)
  {
    //    std::cout << "highLevelSm::turn_on\n";
  }

  void update_motor(updateMotor const&)
  {
    //    std::cout << "highLevelSm::turn_on\n";
  }

  void update_switch(updateSwitch const&)
  {
    //    std::cout << "highLevelSm::turn_on\n";
  }


  void stop_machine(stopMachine const&)
  {
    ipModeMachine.process_event(IPMode::disableIP());
    ipModeMachine.process_event(IPMode::deselectMode());
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
      a_row < Standby   , checkModeSwitch    , ModeSwitch   , &hl::mode_switch                      >,
      a_row < Standby   , enableMove, Move   , &hl::move                     >,
      a_row < Standby   , stopMachine, machineStopped   , &hl::stop_machine                      >,

      a_row < ModeSwitch   , runMotorSM, updateMotorSM   , &hl::motor_sm                     >,
      a_row < ModeSwitch   , enterStandBy, Standby   , &hl::standby                      >,
      a_row < ModeSwitch   , stopMachine, machineStopped   , &hl::stop_machine                      >,
      a_row < ModeSwitch   , updateSwitch, ModeSwitch   , &hl::update_switch                      >,


      a_row < updateMotorSM   , checkModeSwitch    , ModeSwitch   , &hl::mode_switch                       >,
      a_row < updateMotorSM   , enterStandBy, Standby   , &hl::standby                      >,
      a_row < updateMotorSM   , enableMove, Move   , &hl::move                     >,
      a_row < updateMotorSM   , stopMachine, machineStopped   , &hl::stop_machine                      >,
      a_row < updateMotorSM   , updateMotor, updateMotorSM   , &hl::update_motor                      >,

      a_row < Move   , enterStandBy, Standby   , &hl::standby                      >,
      a_row < Move   , stopMachine, machineStopped   , &hl::stop_machine                      >
      //    +---------+-------------+---------+---------------------+----------------------+
      > {};
  // Replaces the default no-transition response.
  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state)
  {
    //std::cout << "no transition from state " << state
    //          << " on event " << typeid(e).name() << std::endl;
  }

  template <class FSM,class Event>
  void exception_caught (Event const&,FSM& fsm,std::exception& )
  {

  }

private:
  boost::shared_ptr<InternalState> state_;
  boost::shared_ptr<InternalState> target_state_;

  boost::shared_ptr<cw_word> control_word_;
  boost::shared_ptr<cw_word> status_word_;

  boost::shared_ptr<double> target_pos_;
  boost::shared_ptr<double> target_vel_;
  boost::shared_ptr<OperationMode> operation_mode_;

  boost::shared_ptr<StatusandControl::commandTargets> targets_;
  OperationMode previous_mode_;

};
// back-end
typedef msm::back::state_machine<highLevelSM_> highLevelSM;
};

#endif // HIGH_LEVEL_SM_H
