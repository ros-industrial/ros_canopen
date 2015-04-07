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

#ifndef HOMING_H
#define HOMING_H

#include <canopen_402/status_and_control.h>
///////
/// \brief m
///
///
///
// the ip mode state machine
namespace msm = boost::msm;
namespace mpl = boost::mpl;


using namespace boost::msm::front;


namespace canopen
{
class HomingSM_ : public msm::front::state_machine_def<HomingSM_>
{
public:
  HomingSM_(){}
  HomingSM_(const boost::shared_ptr<cw_word> &control_word, boost::shared_ptr<sw_word> status_word) : control_word_(control_word), status_word_(status_word)
  {
    homing_mask_.set(SW_Target_reached);
    homing_mask_.set(SW_Operation_specific0);
    homing_mask_.set(SW_Operation_specific1);

    homing_state_ = boost::make_shared<HomingState>(NotStarted);
  }
  struct enableHoming {};
  struct disableHoming {};
  struct selectMode {};
  struct deselectMode {};
  struct runHomingCheck {};

  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {/*std::cout << "entering: IPMode" << std::endl;*/}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: IPMode" << std::endl;*/}

  // The list of FSM states
  struct HomingInactive : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: IPInactive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: IPInactive" << std::endl;*/}

  };
  struct HomingActive : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: IPActive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: IPActive" << std::endl;*/}
  };

  // The list of FSM states
  struct modeDeselected : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: modeDeselected" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: modeDeselected" << std::endl;*/}

  };
  struct modeSelected : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: modeSelected" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: modeSelected" << std::endl;*/}
  };

  struct updateHoming : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: modeSelected" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: modeSelected" << std::endl;*/}
  };

  // the initial state. Must be defined
  typedef mpl::vector<modeDeselected,updateHoming> initial_state;
  // transition actions
  void enable_homing(enableHoming const&)
  {
    control_word_->set(CW_Operation_mode_specific0);
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
    //    std::cout << "IPMode::enable_ip\n";
  }
  void disable_homing(disableHoming const&)
  {
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
  }

  void select_mode(selectMode const&)
  {
    //    std::cout << "IPMode::selectMode\n";
  }

  void update_homing(runHomingCheck const&)
  {
    control_word_->set(CW_Operation_mode_specific0);

    switch ((*status_word_ & homing_mask_).to_ulong())
    {
    //-------------------------------------------------------------//
    // Op_specific1 | Op_specific0 | Target_reached | Description |
    // ------------ | ------------ | -------------- | ----------- |
    // 0 | 0 | 0 | In Progress |
    // 0 | 0 | 1 | Hom.Started |
    // 0 | 1 | 0 | Hom.Attained, target not reached|
    // 0 | 1 | 1 | Hom.Succesful |
    // 1 | 0 | 0 | Hom.Error, vel!=0 |
    // 1 | 0 | 1 | Hom.Error, vel=0 |
    // 1 | 1 | 0 | Hom.Reserved |
    // 1 | 1 | 1 | Hom.Reserved |
    //-------------------------------------------------------------//
    case 0:
      BOOST_THROW_EXCEPTION(std::invalid_argument("Homing in progress"));
      *homing_state_ = Progress;
      break;
    case (1<<SW_Target_reached):
      *homing_state_ = NotStarted;
      BOOST_THROW_EXCEPTION(std::invalid_argument("Homing started"));
      break;
    case (1<<SW_Operation_specific0):
      *homing_state_ = Attained;
      BOOST_THROW_EXCEPTION(std::invalid_argument("Homing attained"));
      break;
    case ((1<<SW_Operation_specific0) | (1<<SW_Target_reached)):
      *homing_state_ = HomingSuccess;
      break;
    case (1<<SW_Operation_specific1):
      *homing_state_ = HomingError;
      BOOST_THROW_EXCEPTION(std::invalid_argument("Error, vel!=0"));
      break;
    case ((1<<SW_Operation_specific1) | (1<<SW_Target_reached)):
      *homing_state_ = HomingError;
      BOOST_THROW_EXCEPTION(std::invalid_argument("Error, vel=0"));
      break;
    case (1<<SW_Operation_specific1 | 1<<SW_Operation_specific0):
    case (1<<SW_Operation_specific1 | 1<<SW_Operation_specific0 | 1<<SW_Target_reached):
      BOOST_THROW_EXCEPTION(std::invalid_argument("Homing reserved"));
      break;
    }
  }
  void deselect_mode(deselectMode const&)
  {
    control_word_->reset(CW_Operation_mode_specific0);
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
  }
  // guard conditions

  typedef HomingSM_ hom; // makes transition table cleaner

  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < modeDeselected   , selectMode    , modeSelected   , &hom::select_mode                       >,

      Row < modeSelected   , none    , HomingInactive   , none, none                       >,
      a_row < modeSelected   , deselectMode    , modeDeselected   , &hom::deselect_mode                       >,

      a_row < HomingActive   , disableHoming, HomingInactive   , &hom::disable_homing                      >,
      a_row < HomingActive   , deselectMode    , modeDeselected   , &hom::deselect_mode                       >,

      a_row < HomingInactive   , enableHoming    , HomingActive   , &hom::enable_homing                       >,
      a_row < HomingInactive   , deselectMode    , modeDeselected   , &hom::deselect_mode                       >,
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < updateHoming   , runHomingCheck    , updateHoming   , &hom::update_homing                       >
      //    +---------+-------------+---------+---------------------+----------------------+
      > {};
  // Replaces the default no-transition response.
  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state)
  {
    //    std::cout << "no transition from state " << state
    //              << " on event " << typeid(e).name() << std::endl;
  }

  template <class FSM,class Event>
  void exception_caught (Event const&,FSM& fsm,std::exception& )
  {

  }
private:
  boost::shared_ptr<cw_word> control_word_;
  boost::shared_ptr<sw_word> status_word_;
  boost::shared_ptr<HomingState> homing_state_;

  std::bitset<16> homing_mask_;

};
// back-end
typedef msm::back::state_machine<HomingSM_> HomingSM;
};
/// */
///
///
#endif // HOMING_H
