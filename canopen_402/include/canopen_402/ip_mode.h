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

#ifndef IP_MODE_H
#define IP_MODE_H

///////
/// \brief m
///
///
///
// the player state machine contains a state which is himself a state machine
// as you see, no need to declare it anywhere so IPMode can be developed separately
// by another team in another module. For simplicity I just declare it inside player
namespace msm = boost::msm;
namespace mpl = boost::mpl;


using namespace boost::msm::front;


namespace canopen
{
class IPMode_ : public msm::front::state_machine_def<IPMode_>
{
public:
  IPMode_(){}
  struct enableIP {};
  struct disableIP {};
  struct selectMode {};
  struct deselectMode {};
  // when IPMode, the CD is loaded and we are in either pause or IPMode (duh)
  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {std::cout << "entering: IPMode" << std::endl;}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {std::cout << "leaving: IPMode" << std::endl;}

  // The list of FSM states
  struct IPInactive : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: IPInactive" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: IPInactive" << std::endl;}

  };
  struct IPActive : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: IPActive" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: IPActive" << std::endl;}
  };

  // The list of FSM states
  struct modeDeselected : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: modeDeselected" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: modeDeselected" << std::endl;}

  };
  struct modeSelected : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: modeSelected" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: modeSelected" << std::endl;}
  };

  // the initial state. Must be defined
  typedef modeDeselected initial_state;
  // transition actions
  void enable_ip(enableIP const&)
  {
    std::cout << "IPMode::enable_ip\n";
  }
  void disable_ip(disableIP const&)
  {
    std::cout << "IPMode::read_status\n";
  }

  void select_mode(selectMode const&)
  {
    std::cout << "IPMode::selectMode\n";
  }
  void deselect_mode(deselectMode const&)
  {
    std::cout << "IPMode::deselectMode\n";
  }
  // guard conditions

  typedef IPMode_ ip; // makes transition table cleaner
  // Transition table for IPMode
  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < modeDeselected   , selectMode    , modeSelected   , &ip::select_mode                       >,

      Row < modeSelected   , none    , IPInactive   , none, none                       >,
      a_row < modeSelected   , deselectMode    , modeDeselected   , &ip::deselect_mode                       >,

      a_row < IPActive   , disableIP, IPInactive   , &ip::disable_ip                      >,
      a_row < IPActive   , deselectMode    , modeDeselected   , &ip::deselect_mode                       >,

      a_row < IPInactive   , enableIP    , IPActive   , &ip::enable_ip                       >,
      a_row < IPInactive   , deselectMode    , modeDeselected   , &ip::deselect_mode                       >
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
typedef msm::back::state_machine<IPMode_> IPMode;
};
/// */
///
///
#endif // IP_MODE_H
