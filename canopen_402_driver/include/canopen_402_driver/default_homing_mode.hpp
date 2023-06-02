#ifndef DEFAULT_HOMING_MODE_HPP
#define DEFAULT_HOMING_MODE_HPP
#include <mutex>
#include "homing_mode.hpp"
#include "lely_motion_controller_bridge.hpp"

namespace ros2_canopen
{

class DefaultHomingMode : public HomingMode
{
  const uint16_t index = 0x6098;
  std::shared_ptr<LelyDriverBridge> driver;

  std::atomic<bool> execute_;

  std::mutex mutex_;
  std::condition_variable cond_;
  uint16_t status_;

  enum SW_masks
  {
    MASK_Reached = (1 << State402::SW_Target_reached),
    MASK_Attained = (1 << SW_Attained),
    MASK_Error = (1 << SW_Error),
  };
  bool error(const std::string & msg)
  {
    execute_ = false;
    std::cout << msg << std::endl;
    return false;
  }

public:
  DefaultHomingMode(std::shared_ptr<LelyDriverBridge> driver) { this->driver = driver; }
  virtual bool start();
  virtual bool read(const uint16_t & sw);
  virtual bool write(OpModeAccesser & cw);

  virtual bool executeHoming();
};
}  // namespace ros2_canopen
#endif  // DEFAULT_HOMING_MODE_HPP
