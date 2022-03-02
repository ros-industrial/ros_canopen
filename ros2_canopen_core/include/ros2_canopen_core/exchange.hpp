#ifndef EXCHANGE_HPP
#define EXCHANGE_HPP

enum CODataTypes
{
  CODataUnkown = 0,
  COData8 = 8,
  COData16 = 16,
  COData32 = 32
};

struct COData
{
public:
  uint16_t index_;
  uint8_t subindex_;
  uint32_t data_;
  CODataTypes type_;
};

#endif  // EXCHANGE_HPP