#ifndef CANOPEN_402_DRIVER_STATE_HPP
#define CANOPEN_402_DRIVER_STATE_HPP

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace ros2_canopen
{
class State402
{
public:
  enum StatusWord
  {
    SW_Ready_To_Switch_On = 0,
    SW_Switched_On = 1,
    SW_Operation_enabled = 2,
    SW_Fault = 3,
    SW_Voltage_enabled = 4,
    SW_Quick_stop = 5,
    SW_Switch_on_disabled = 6,
    SW_Warning = 7,
    SW_Manufacturer_specific0 = 8,
    SW_Remote = 9,
    SW_Target_reached = 10,
    SW_Internal_limit = 11,
    SW_Operation_mode_specific0 = 12,
    SW_Operation_mode_specific1 = 13,
    SW_Manufacturer_specific1 = 14,
    SW_Manufacturer_specific2 = 15
  };
  enum InternalState
  {
    Unknown = 0,
    Start = 0,
    Not_Ready_To_Switch_On = 1,
    Switch_On_Disabled = 2,
    Ready_To_Switch_On = 3,
    Switched_On = 4,
    Operation_Enable = 5,
    Quick_Stop_Active = 6,
    Fault_Reaction_Active = 7,
    Fault = 8,
  };
  InternalState getState();
  InternalState read(uint16_t sw);
  bool waitForNewState(
    const std::chrono::steady_clock::time_point & abstime, InternalState & state);
  State402() : state_(Unknown) {}

private:
  std::condition_variable cond_;
  std::mutex mutex_;
  InternalState state_;
};
}  // namespace ros2_canopen

#endif  // CANOPEN_402_DRIVER_STATE_HPP
