#ifndef STATUS_AND_CONTROL_MACHINE_H
#define STATUS_AND_CONTROL_MACHINE_H

#include <bitset>

namespace canopen
{

class StatusandControlSM_ : public msm::front::state_machine_def<StatusandControlSM_>
{
public:
  typedef std::bitset<16> word_bitset;
  typedef boost::shared_ptr<word_bitset> word_bitset_ptr;

private:
  word_bitset_ptr control_word_bitset;
  word_bitset_ptr status_word_bitset;

public:

  typedef Not_Ready_To_Switch_On_State initial_state;
  // defined events from transitioning inside the 402 state machine
  struct get_status {};
  struct send_control {};

  word_bitset_ptr getStatusWord()
  {
    return control_word_bitset;
  }

  word_bitset_ptr getControlWorld()
  {
    return control_word_bitset;
  }

  void setStatusWord(word_bitset_ptr &word_bitset_new)
  {
    status_word_bitset = word_bitset_new;
  }

  void setControlWord(word_bitset_ptr &word_bitset_new)
  {
    control_word_bitset = word_bitset_new;
  }

  void motorShutdown()
  {
    control_word_bitset->reset(CW_Switch_On);
    control_word_bitset->set(CW_Enable_Voltage);
    control_word_bitset->set(CW_Quick_Stop);
    control_word_bitset->reset(CW_Fault_Reset);
    control_word_bitset->reset(CW_Enable_Operation);
    control_word_bitset->reset(CW_Operation_mode_specific0);
    control_word_bitset->reset(CW_Operation_mode_specific1);
    control_word_bitset->reset(CW_Operation_mode_specific2);
  }

  void motorSwitchOn()
  {
    control_word_bitset->set(CW_Switch_On);
    control_word_bitset->set(CW_Enable_Voltage);
    control_word_bitset->set(CW_Quick_Stop);
    control_word_bitset->reset(CW_Fault_Reset);
    control_word_bitset->reset(CW_Enable_Operation);
    control_word_bitset->reset(CW_Operation_mode_specific0);
    control_word_bitset->reset(CW_Operation_mode_specific1);
    control_word_bitset->reset(CW_Operation_mode_specific2);
  }

  void motorSwitchOnandEnableOp()
  {
    control_word_bitset->set(CW_Switch_On);
    control_word_bitset->set(CW_Enable_Voltage);
    control_word_bitset->set(CW_Quick_Stop);
    control_word_bitset->reset(CW_Fault_Reset);
    control_word_bitset->set(CW_Enable_Operation);
    control_word_bitset->reset(CW_Operation_mode_specific0);
    control_word_bitset->reset(CW_Operation_mode_specific1);
    control_word_bitset->reset(CW_Operation_mode_specific2);
  }

  void motorDisableVoltage()
  {
    control_word_bitset->reset(CW_Switch_On);
    control_word_bitset->reset(CW_Enable_Voltage);
    control_word_bitset->reset(CW_Quick_Stop);
    control_word_bitset->reset(CW_Fault_Reset);
    control_word_bitset->reset(CW_Enable_Operation);
    control_word_bitset->reset(CW_Operation_mode_specific0);
    control_word_bitset->reset(CW_Operation_mode_specific1);
    control_word_bitset->reset(CW_Operation_mode_specific2);
  }

  void motorQuickStop()
  {
    control_word_bitset->reset(CW_Switch_On);
    control_word_bitset->set(CW_Enable_Voltage);
    control_word_bitset->reset(CW_Quick_Stop);
    control_word_bitset->reset(CW_Fault_Reset);
    control_word_bitset->reset(CW_Enable_Operation);
    control_word_bitset->reset(CW_Operation_mode_specific0);
    control_word_bitset->reset(CW_Operation_mode_specific1);
    control_word_bitset->reset(CW_Operation_mode_specific2);
  }

  void motorDisableOp()
  {
    control_word_bitset->set(CW_Switch_On);
    control_word_bitset->set(CW_Enable_Voltage);
    control_word_bitset->set(CW_Quick_Stop);
    control_word_bitset->reset(CW_Fault_Reset);
    control_word_bitset->reset(CW_Enable_Operation);
    control_word_bitset->reset(CW_Operation_mode_specific0);
    control_word_bitset->reset(CW_Operation_mode_specific1);
    control_word_bitset->reset(CW_Operation_mode_specific2);
  }

  void motorEnableOp()
  {
    control_word_bitset->set(CW_Switch_On);
    control_word_bitset->set(CW_Enable_Voltage);
    control_word_bitset->set(CW_Quick_Stop);
    control_word_bitset->reset(CW_Fault_Reset);
    control_word_bitset->set(CW_Enable_Operation);
    control_word_bitset->reset(CW_Operation_mode_specific0);
    control_word_bitset->reset(CW_Operation_mode_specific1);
    control_word_bitset->reset(CW_Operation_mode_specific2);
    control_word_bitset->reset(CW_Halt);
  }

  void motorFaultReset()
  {
    control_word_bitset->set(CW_Fault_Reset);
    control_word_bitset->reset(CW_Switch_On);
    control_word_bitset->reset(CW_Enable_Voltage);
    control_word_bitset->set(CW_Quick_Stop);
    control_word_bitset->reset(CW_Enable_Operation);
    control_word_bitset->reset(CW_Operation_mode_specific0);
    control_word_bitset->reset(CW_Operation_mode_specific1);
    control_word_bitset->reset(CW_Operation_mode_specific2);
  }
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

  typedef StatusandControlSM_ m; // makes transition table cleaner

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


typedef msm::back::state_machine<StatusandControlSM_> StatusandControlSM;
};

#endif // STATE_MACHINE_H



#endif // STATUS_AND_CONTROL_MACHINE_H
