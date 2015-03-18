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

namespace msm = boost::msm;
namespace mpl = boost::mpl;


using namespace boost::msm::front;

namespace canopen
{
class highLevelSm_ : public msm::front::state_machine_def<highLevelSm_>
{
public:
  highLevelSm_(){}
  struct turnOn {};
  struct turnOff {};

  class OnSm_ : public msm::front::state_machine_def<OnSm_>
  {
  public:
    OnSm_(){}
    struct checkUpProcedure {};
    struct checkModeSwitch {};
    struct enableDrive {};

    // when highLevelSm, the CD is loaded and we are in either pause or highLevelSm (duh)
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "entering: OnSm" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "leaving: OnSm" << std::endl;}

    // The list of FSM states
    struct StartUp : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {std::cout << "starting: StartUp" << std::endl;}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {std::cout << "finishing: StartUp" << std::endl;}

    };
    // The list of FSM states
    struct CheckUp : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {std::cout << "starting: CheckUp" << std::endl;}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {std::cout << "finishing: CheckUp" << std::endl;}

    };
    struct ModeSwitch : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {std::cout << "starting: ModeSwitch" << std::endl;}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {std::cout << "finishing: ModeSwitch" << std::endl;}
    };

    struct Drive : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const&,FSM& ) {std::cout << "starting: Drive" << std::endl;}
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& ) {std::cout << "finishing: Drive" << std::endl;}
    };

    // the initial state. Must be defined
    typedef StartUp initial_state;
    // transition actions
    void check_up(checkUpProcedure const&)
    {
      std::cout << "OnSm::check_up" << std::endl;
    }
    void mode_switch(checkModeSwitch const&)
    {
      std::cout << "OnSm::switch_mode" << std::endl;
    }

    void drive(enableDrive const&)
    {
      std::cout << "OnSm::drive" << std::endl;
    }
    // guard conditions

    typedef OnSm_ on; // makes transition table cleaner
    // Transition table for OnSm
    struct transition_table : mpl::vector<
        //      Start     Event         Next      Action               Guard
        //    +---------+-------------+---------+---------------------+----------------------+
        Row < StartUp   , none    , CheckUp   , none, none                       >,
        a_row < CheckUp   , checkModeSwitch    , ModeSwitch   , &on::mode_switch                       >,
        a_row < ModeSwitch   , enableDrive, Drive   , &on::drive                     >,
        a_row < Drive   , checkUpProcedure, CheckUp   , &on::check_up                      >
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
  typedef msm::back::state_machine<OnSm_> OnSm;

  // when highLevelSm, the CD is loaded and we are in either pause or highLevelSm (duh)
  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {std::cout << "entering: highLevelSm" << std::endl;}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {std::cout << "leaving: highLevelSm" << std::endl;}

  // The list of FSM states
  struct Off : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: Off" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: Off" << std::endl;}

  };
  struct On : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {std::cout << "starting: On" << std::endl;}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {std::cout << "finishing: On" << std::endl;}
  };

  // the initial state. Must be defined
  typedef Off initial_state;
  // transition actions
  void turn_on(turnOn const&)
  {
    std::cout << "highLevelSm::turn_on\n";
  }

  void turn_off(turnOff const&)
  {
    std::cout << "highLevelSm::read_status\n";
  }
  // guard conditions

  typedef highLevelSm_ hl; // makes transition table cleaner
  // Transition table for highLevelSm
  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < Off   , turnOn    , OnSm   , &hl::turn_on                       >,
      a_row < OnSm   , turnOff, Off   , &hl::turn_off                      >
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
typedef msm::back::state_machine<highLevelSm_> highLevelSm;
};
/// */
///
///


#endif // HIGH_LEVEL_SM_H
