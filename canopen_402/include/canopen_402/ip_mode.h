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
namespace canopen
{
class IPMode_ : public msm::front::state_machine_def<IPMode_>
{
public:
  IPMode_(){}
  struct enableIP {};
  struct disableIP {};
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

  // the initial state. Must be defined
  typedef IPInactive initial_state;
  // transition actions
  void enable_ip(enableIP const&)
  {
    std::cout << "IPMode::enable_ip\n";
  }
  void disable_ip(disableIP const&)
  {
    std::cout << "IPMode::read_status\n";
  }
  // guard conditions

  typedef IPMode_ pl; // makes transition table cleaner
  // Transition table for IPMode
  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < IPInactive   , enableIP    , IPActive   , &pl::enable_ip                       >,
      a_row < IPActive   , disableIP, IPInactive   , &pl::disable_ip                      >
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
