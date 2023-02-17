//    Copyright 2022 Harshavadan Deshpande
//                   Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.
#ifndef EXCHANGE_HPP
#define EXCHANGE_HPP

namespace ros2_canopen
{
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

struct COEmcy
{
public:
  uint16_t eec;
  uint8_t er;
  uint8_t msef[5];
};
}  // namespace ros2_canopen

#endif  // EXCHANGE_HPP
