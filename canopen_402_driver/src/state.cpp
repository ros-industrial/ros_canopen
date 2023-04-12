#include "canopen_402_driver/state.hpp"
#include <mutex>
using namespace ros2_canopen;

State402::InternalState State402::getState()
{
  std::scoped_lock lock(mutex_);
  return state_;
}

State402::InternalState State402::read(uint16_t sw)
{
  static const uint16_t r = (1 << SW_Ready_To_Switch_On);
  static const uint16_t s = (1 << SW_Switched_On);
  static const uint16_t o = (1 << SW_Operation_enabled);
  static const uint16_t f = (1 << SW_Fault);
  static const uint16_t q = (1 << SW_Quick_stop);
  static const uint16_t d = (1 << SW_Switch_on_disabled);

  InternalState new_state = Unknown;

  uint16_t state = sw & (d | q | f | o | s | r);
  switch (state)
  {
    //   ( d | q | f | o | s | r ):
    case (0 | 0 | 0 | 0 | 0 | 0):
    case (0 | q | 0 | 0 | 0 | 0):
      // std::cout << "Not_Ready_To_Switch_On" << std::endl;
      new_state = Not_Ready_To_Switch_On;
      break;

    case (d | 0 | 0 | 0 | 0 | 0):
    case (d | q | 0 | 0 | 0 | 0):
      // std::cout << "Switch_On_Disabled" << std::endl;
      new_state = Switch_On_Disabled;
      break;

    case (0 | q | 0 | 0 | 0 | r):
      // std::cout << "Ready_To_Switch_On" << std::endl;
      new_state = Ready_To_Switch_On;
      break;

    case (0 | q | 0 | 0 | s | r):
      new_state = Switched_On;
      break;

    case (0 | q | 0 | o | s | r):
      new_state = Operation_Enable;
      break;

    case (0 | 0 | 0 | o | s | r):
      new_state = Quick_Stop_Active;
      break;

    case (0 | 0 | f | o | s | r):
    case (0 | q | f | o | s | r):
      new_state = Fault_Reaction_Active;
      break;

    case (0 | 0 | f | 0 | 0 | 0):
    case (0 | q | f | 0 | 0 | 0):
      new_state = Fault;
      break;

    default:
      /// @todo Throw error here.
      break;
  }
  std::scoped_lock lock(mutex_);
  if (new_state != state_)
  {
    state_ = new_state;
    cond_.notify_all();
  }
  return state_;
}
bool State402::waitForNewState(
  const std::chrono::steady_clock::time_point & abstime, State402::InternalState & state)
{
  std::unique_lock lock(mutex_);
  while (state_ == state && cond_.wait_until(lock, abstime) == std::cv_status::no_timeout)
  {
  }
  bool res = state != state_;
  state = state_;
  return res;
}
