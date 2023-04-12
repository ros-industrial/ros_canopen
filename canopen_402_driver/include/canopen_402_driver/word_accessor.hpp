#ifndef WORD_ACCESSOR_HPP
#define WORD_ACCESSOR_HPP

#include <cstdint>

namespace ros2_canopen
{
template <uint16_t MASK>
class WordAccessor
{
  uint16_t & word_;

public:
  WordAccessor(uint16_t & word) : word_(word) {}
  bool set(uint8_t bit)
  {
    uint16_t val = MASK & (1 << bit);
    word_ |= val;
    return val;
  }
  bool reset(uint8_t bit)
  {
    uint16_t val = MASK & (1 << bit);
    word_ &= ~val;
    return val;
  }
  bool get(uint8_t bit) const { return word_ & (1 << bit); }
  uint16_t get() const { return word_ & MASK; }
  WordAccessor & operator=(const uint16_t & val)
  {
    word_ = (word_ & ~MASK) | (val & MASK);
    return *this;
  }
};
}  // namespace ros2_canopen

#endif  // WORD_ACCESSOR_HPP
