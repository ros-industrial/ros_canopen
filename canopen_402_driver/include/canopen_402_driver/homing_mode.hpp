#ifndef HOMING_MODE_HPP
#define HOMING_MODE_HPP
#include "base.hpp"
#include "mode.hpp"

namespace ros2_canopen
{
class HomingMode : public Mode
{
protected:
  enum SW_bits
  {
    SW_Attained = State402::SW_Operation_mode_specific0,
    SW_Error = State402::SW_Operation_mode_specific1,
  };
  enum CW_bits
  {
    CW_StartHoming = Command402::CW_Operation_mode_specific0,
  };

public:
  HomingMode() : Mode(MotorBase::Homing) {}
  virtual bool executeHoming() = 0;
};
}  // namespace ros2_canopen

#endif  // HOMING_MODE_HPP
