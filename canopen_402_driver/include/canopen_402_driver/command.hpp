#ifndef CANOPEN_402_DRIVER_COMMAND_HPP
#define CANOPEN_402_DRIVER_COMMAND_HPP

#include <boost/container/flat_map.hpp>
#include <cstdint>
#include <utility>
#include "state.hpp"

namespace ros2_canopen
{
class Command402
{
  struct Op
  {
    uint16_t to_set_;
    uint16_t to_reset_;
    Op(uint16_t to_set, uint16_t to_reset) : to_set_(to_set), to_reset_(to_reset) {}
    void operator()(uint16_t & val) const { val = (val & ~to_reset_) | to_set_; }
  };
  class TransitionTable
  {
    boost::container::flat_map<std::pair<State402::InternalState, State402::InternalState>, Op>
      transitions_;
    void add(const State402::InternalState & from, const State402::InternalState & to, Op op)
    {
      transitions_.insert(std::make_pair(std::make_pair(from, to), op));
    }

  public:
    TransitionTable();
    const Op & get(const State402::InternalState & from, const State402::InternalState & to) const
    {
      return transitions_.at(std::make_pair(from, to));
    }
  };
  static const TransitionTable transitions_;
  static State402::InternalState nextStateForEnabling(State402::InternalState state);
  Command402();

public:
  enum ControlWord
  {
    CW_Switch_On = 0,
    CW_Enable_Voltage = 1,
    CW_Quick_Stop = 2,
    CW_Enable_Operation = 3,
    CW_Operation_mode_specific0 = 4,
    CW_Operation_mode_specific1 = 5,
    CW_Operation_mode_specific2 = 6,
    CW_Fault_Reset = 7,
    CW_Halt = 8,
    CW_Operation_mode_specific3 = 9,
    // CW_Reserved1=10,
    CW_Manufacturer_specific0 = 11,
    CW_Manufacturer_specific1 = 12,
    CW_Manufacturer_specific2 = 13,
    CW_Manufacturer_specific3 = 14,
    CW_Manufacturer_specific4 = 15,
  };
  static bool setTransition(
    uint16_t & cw, const State402::InternalState & from, const State402::InternalState & to,
    State402::InternalState * next);
};
}  // namespace ros2_canopen

#endif  // CANOPEN_402_DRIVER_COMMAND_HPP
