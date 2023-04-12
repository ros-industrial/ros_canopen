#include "canopen_402_driver/command.hpp"
using namespace ros2_canopen;

const Command402::TransitionTable Command402::transitions_;

Command402::TransitionTable::TransitionTable()
{
  typedef State402 s;

  transitions_.reserve(32);

  Op disable_voltage(0, (1 << CW_Fault_Reset) | (1 << CW_Enable_Voltage));
  /* 7*/ add(s::Ready_To_Switch_On, s::Switch_On_Disabled, disable_voltage);
  /* 9*/ add(s::Operation_Enable, s::Switch_On_Disabled, disable_voltage);
  /*10*/ add(s::Switched_On, s::Switch_On_Disabled, disable_voltage);
  /*12*/ add(s::Quick_Stop_Active, s::Switch_On_Disabled, disable_voltage);

  Op automatic(0, 0);
  /* 0*/ add(s::Start, s::Not_Ready_To_Switch_On, automatic);
  /* 1*/ add(s::Not_Ready_To_Switch_On, s::Switch_On_Disabled, automatic);
  /*14*/ add(s::Fault_Reaction_Active, s::Fault, automatic);

  Op shutdown(
    (1 << CW_Quick_Stop) | (1 << CW_Enable_Voltage), (1 << CW_Fault_Reset) | (1 << CW_Switch_On));
  /* 2*/ add(s::Switch_On_Disabled, s::Ready_To_Switch_On, shutdown);
  /* 6*/ add(s::Switched_On, s::Ready_To_Switch_On, shutdown);
  /* 8*/ add(s::Operation_Enable, s::Ready_To_Switch_On, shutdown);

  Op switch_on(
    (1 << CW_Quick_Stop) | (1 << CW_Enable_Voltage) | (1 << CW_Switch_On),
    (1 << CW_Fault_Reset) | (1 << CW_Enable_Operation));
  /* 3*/ add(s::Ready_To_Switch_On, s::Switched_On, switch_on);
  /* 5*/ add(s::Operation_Enable, s::Switched_On, switch_on);

  Op enable_operation(
    (1 << CW_Quick_Stop) | (1 << CW_Enable_Voltage) | (1 << CW_Switch_On) |
      (1 << CW_Enable_Operation),
    (1 << CW_Fault_Reset));
  /* 4*/ add(s::Switched_On, s::Operation_Enable, enable_operation);
  /*16*/ add(s::Quick_Stop_Active, s::Operation_Enable, enable_operation);

  Op quickstop((1 << CW_Enable_Voltage), (1 << CW_Fault_Reset) | (1 << CW_Quick_Stop));
  /* 7*/ add(
    s::Ready_To_Switch_On, s::Quick_Stop_Active, quickstop);    // transit to Switch_On_Disabled
  /*10*/ add(s::Switched_On, s::Quick_Stop_Active, quickstop);  // transit to Switch_On_Disabled
  /*11*/ add(s::Operation_Enable, s::Quick_Stop_Active, quickstop);

  // fault reset
  /*15*/ add(s::Fault, s::Switch_On_Disabled, Op((1 << CW_Fault_Reset), 0));
}
State402::InternalState Command402::nextStateForEnabling(State402::InternalState state)
{
  switch (state)
  {
    case State402::Start:
      return State402::Not_Ready_To_Switch_On;

    case State402::Fault:
    case State402::Not_Ready_To_Switch_On:
      return State402::Switch_On_Disabled;

    case State402::Switch_On_Disabled:
      return State402::Ready_To_Switch_On;

    case State402::Ready_To_Switch_On:
      return State402::Switched_On;

    case State402::Switched_On:
    case State402::Quick_Stop_Active:
    case State402::Operation_Enable:
      return State402::Operation_Enable;

    case State402::Fault_Reaction_Active:
      return State402::Fault;
  }
  throw std::out_of_range("state value is illegal");
}

bool Command402::setTransition(
  uint16_t & cw, const State402::InternalState & from, const State402::InternalState & to,
  State402::InternalState * next)
{
  try
  {
    if (from != to)
    {
      State402::InternalState hop = to;
      if (next)
      {
        if (to == State402::Operation_Enable) hop = nextStateForEnabling(from);
        *next = hop;
      }
      transitions_.get(from, hop)(cw);
    }
    return true;
  }
  catch (...)
  {
    /// @todo Print error here.
  }
  return false;
}
