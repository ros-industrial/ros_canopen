#ifndef MODE_FORWARD_HELPER_HPP
#define MODE_FORWARD_HELPER_HPP

#include <cstdint>
#include <memory>
#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "mode_target_helper.hpp"

namespace ros2_canopen
{

template <uint16_t ID, typename TYPE, uint16_t OBJ, uint8_t SUB, uint16_t CW_MASK>
class ModeForwardHelper : public ModeTargetHelper<TYPE>
{
  std::shared_ptr<LelyDriverBridge> driver;

public:
  ModeForwardHelper(std::shared_ptr<LelyDriverBridge> driver) : ModeTargetHelper<TYPE>(ID)
  {
    this->driver = driver;
  }
  virtual bool read(const uint16_t & sw) { return true; }
  virtual bool write(Mode::OpModeAccesser & cw)
  {
    if (this->hasTarget())
    {
      cw = cw.get() | CW_MASK;

      driver->universal_set_value<TYPE>(OBJ, SUB, this->getTarget());
      return true;
    }
    else
    {
      cw = cw.get() & ~CW_MASK;
      return false;
    }
  }
};
}  // namespace ros2_canopen

#endif  // MODE_FORWARD_HELPER_HPP
