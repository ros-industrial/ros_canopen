#ifndef MODE_TARGET_HELPER_HPP
#define MODE_TARGET_HELPER_HPP

#include <atomic>
#include <boost/numeric/conversion/cast.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "canopen_402_driver/mode.hpp"

namespace ros2_canopen
{

template <typename T>
class ModeTargetHelper : public Mode
{
  T target_;
  std::atomic<bool> has_target_;

public:
  ModeTargetHelper(uint16_t mode) : Mode(mode) {}
  bool hasTarget() { return has_target_; }
  T getTarget() { return target_; }
  virtual bool setTarget(const double & val)
  {
    if (std::isnan(val))
    {
      std::cout << "canopen_402 target command is not a number" << std::endl;
      return false;
    }

    using boost::numeric_cast;
    using boost::numeric::negative_overflow;
    using boost::numeric::positive_overflow;

    try
    {
      target_ = numeric_cast<T>(val);
    }
    catch (negative_overflow &)
    {
      std::cout << "canopen_402 Command " << val
                << " does not fit into target, clamping to min limit" << std::endl;
      target_ = std::numeric_limits<T>::min();
    }
    catch (positive_overflow &)
    {
      std::cout << "canopen_402 Command " << val
                << " does not fit into target, clamping to max limit" << std::endl;
      target_ = std::numeric_limits<T>::max();
    }
    catch (...)
    {
      std::cout << "canopen_402 Was not able to cast command " << val << std::endl;
      return false;
    }

    has_target_ = true;
    return true;
  }
  virtual bool start()
  {
    has_target_ = false;
    return true;
  }
};
}  // namespace ros2_canopen

#endif  // MODE_TARGET_HELPER_HPP
