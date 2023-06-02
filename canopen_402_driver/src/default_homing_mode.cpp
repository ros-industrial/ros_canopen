#include "canopen_402_driver/default_homing_mode.hpp"
using namespace ros2_canopen;

template <uint16_t mask, uint16_t not_equal>
struct masked_status_not_equal
{
  uint16_t & status_;
  masked_status_not_equal(uint16_t & status) : status_(status) {}
  bool operator()() const { return (status_ & mask) != not_equal; }
};
bool DefaultHomingMode::start()
{
  execute_ = false;
  return read(0);
}
bool DefaultHomingMode::read(const uint16_t & sw)
{
  std::scoped_lock lock(mutex_);
  uint16_t old = status_;
  status_ = sw & (MASK_Reached | MASK_Attained | MASK_Error);
  if (old != status_)
  {
    cond_.notify_all();
  }
  return true;
}
bool DefaultHomingMode::write(Mode::OpModeAccesser & cw)
{
  cw = 0;
  if (execute_)
  {
    cw.set(CW_StartHoming);
    return true;
  }
  return false;
}

bool DefaultHomingMode::executeHoming()
{
  int hmode = driver->universal_get_value<int8_t>(index, 0x0);
  if (hmode == 0)
  {
    return true;
  }
  /// @ get abs time from canopen_master
  std::chrono::steady_clock::time_point prepare_time =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  // ensure homing is not running
  std::unique_lock lock(mutex_);
  if (!cond_.wait_until(
        lock, prepare_time, masked_status_not_equal<MASK_Error | MASK_Reached, 0>(status_)))
  {
    return error("could not prepare homing");
  }
  if (status_ & MASK_Error)
  {
    return error("homing error before start");
  }

  execute_ = true;

  // ensure start
  if (!cond_.wait_until(
        lock, prepare_time,
        masked_status_not_equal<MASK_Error | MASK_Attained | MASK_Reached, MASK_Reached>(status_)))
  {
    return error("homing did not start");
  }
  if (status_ & MASK_Error)
  {
    return error("homing error at start");
  }

  std::chrono::steady_clock::time_point finish_time =
    std::chrono::steady_clock::now() + std::chrono::seconds(10);  //

  // wait for attained
  if (!cond_.wait_until(
        lock, finish_time, masked_status_not_equal<MASK_Error | MASK_Attained, 0>(status_)))
  {
    return error("homing not attained");
  }
  if (status_ & MASK_Error)
  {
    return error("homing error during process");
  }

  // wait for motion stop
  if (!cond_.wait_until(
        lock, finish_time, masked_status_not_equal<MASK_Error | MASK_Reached, 0>(status_)))
  {
    return error("homing did not stop");
  }
  if (status_ & MASK_Error)
  {
    return error("homing error during stop");
  }

  if ((status_ & MASK_Reached) && (status_ & MASK_Attained))
  {
    execute_ = false;
    return true;
  }

  return error("something went wrong while homing");
}
