#ifndef STATUS_AND_CONTROL_H
#define STATUS_AND_CONTROL_H

#include <string>
#include <vector>
///////
/// \brief m
///
///
///
// the player state machine contains a state which is himself a state machine
// as you see, no need to declare it anywhere so StatusandControl can be developed separately
// by another team in another module. For simplicity I just declare it inside player
namespace canopen
{
typedef std::bitset<16> sw_word;
typedef std::bitset<16> cw_word;

class StatusandControl_ : public msm::front::state_machine_def<StatusandControl_>
{
public:
  StatusandControl_(){}
  StatusandControl_(boost::shared_ptr<sw_word> status_word, boost::shared_ptr<cw_word> control_word) : status_word_(status_word), control_word_(control_word)
  {
  }

  struct newStatusWord {};
  struct newControlWord {};
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
  void write_control(newStatusWord const&)
  {
    std::cout << "StatusandControl::write_control\n";
    std::cout << "STATUS_WRITE:" << (*status_word_.get()) << std::endl;
    std::cout << "CONTROL_WRITE:" << (*control_word_.get()) << std::endl;
  }
  void read_status(newControlWord const&)
  {
    std::cout << "STATUS_READ:" << (*status_word_.get()) << std::endl;
    std::cout << "CONTROL_READ:" << (*control_word_.get()) << std::endl;
    std::cout << "StatusandControl::read_status\n";
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
    std::cout << "no transition from state " << state
              << " on event " << typeid(e).name() << std::endl;
  }

private:
  boost::shared_ptr<sw_word> status_word_;
  boost::shared_ptr<cw_word> control_word_;

};
// back-end
typedef msm::back::state_machine<StatusandControl_> StatusandControl;
};
/// */
///
#endif // STATUS_AND_CONTROL_H
