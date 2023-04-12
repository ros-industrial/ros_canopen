#ifndef CANOPEN_402_DRIVER_MODE_HPP
#define CANOPEN_402_DRIVER_MODE_HPP

#include <cstdint>
#include <memory>
#include "command.hpp"
#include "state.hpp"
#include "word_accessor.hpp"

namespace ros2_canopen
{
class Mode
{
public:
  const uint16_t mode_id_;
  Mode(uint16_t id) : mode_id_(id) {}
  typedef WordAccessor<
    (1 << Command402::CW_Operation_mode_specific0) |
    (1 << Command402::CW_Operation_mode_specific1) |
    (1 << Command402::CW_Operation_mode_specific2) | (1 << Command402::CW_Operation_mode_specific3)>
    OpModeAccesser;
  virtual bool start() = 0;
  virtual bool read(const uint16_t & sw) = 0;
  virtual bool write(OpModeAccesser & cw) = 0;
  virtual bool setTarget(const double & val) { return false; }
  virtual ~Mode() {}
};
typedef std::shared_ptr<Mode> ModeSharedPtr;
}  // namespace ros2_canopen

#endif  // CANOPEN_402_DRIVER_MODE_HPP
