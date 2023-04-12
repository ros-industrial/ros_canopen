#ifndef MODE_FORWARD_HELPER_HPP
#define MODE_FORWARD_HELPER_HPP

#include <cstdint>
#include <memory>
#include "lely_motion_controller_bridge.hpp"
#include "mode_target_helper.hpp"

namespace ros2_canopen
{

template <uint16_t ID, typename TYPE, CODataTypes TPY, uint16_t OBJ, uint8_t SUB, uint16_t CW_MASK>
class ModeForwardHelper : public ModeTargetHelper<TYPE>
{
  std::shared_ptr<LelyMotionControllerBridge> driver;
  std::shared_ptr<RemoteObject> obj;

public:
  ModeForwardHelper(std::shared_ptr<LelyMotionControllerBridge> driver) : ModeTargetHelper<TYPE>(ID)
  {
    this->obj = driver->create_remote_obj(OBJ, SUB, TPY);
    this->driver = driver;
  }
  virtual bool read(const uint16_t & sw) { return true; }
  virtual bool write(Mode::OpModeAccesser & cw)
  {
    if (this->hasTarget())
    {
      cw = cw.get() | CW_MASK;

      driver->set_remote_obj(obj, this->getTarget());
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
